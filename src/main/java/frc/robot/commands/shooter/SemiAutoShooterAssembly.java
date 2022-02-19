package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.devices.Lemonlight;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.lists.AxisPriorities;


/**
 * Command for running the shooter in semi auto mode.
 */
public class SemiAutoShooterAssembly extends FullAutoShooterAssembly {

    // OI
    private OIButton shootButton;
    private OIButton.PrioritizedButton prioritizedShootButton;

    /**
     * Command for running the shooter in semi auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     * @param drivetrain The drivetrain subsystem.
     * @param limelight The limelight device.
     * @param shootButton The button to enable the shooter.
     */
    public SemiAutoShooterAssembly(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        Lemonlight limelight,
        OIButton shootButton) {

        super(shooter, conveyor, drivetrain, limelight);
        super.initialize();
        this.shootButton = shootButton;

        alignPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);

        // TODO - Set these, including the constants
        alignPID.setTolerance(Shooter.TARGET_HORIZONTAL_ACCURACY, 1);
        movePID.setTolerance(1, 1);
        alignPID.setSetpoint(0);
        movePID.setSetpoint(Shooter.IDEAL_SHOOTING_DISTANCE);

        addRequirements(shooter, drivetrain, conveyor);
    }

    @Override
    public void initialize() {
        shooter.stop();
        teamColor = getTeamColor();
        prioritizedShootButton = shootButton.prioritize(AxisPriorities.DEFAULT);
        alignPID.reset();
        movePID.reset();
        shootDelayTimer.start();
    }

    @Override
    public void execute() {
        limelightHasTarget = limelight.hasTarget();
        limelightDistanceEstimate = limelight.getLimelightDistanceEstimateIN();
        smoothedHorizontalOffset = limelight.getSmoothedHorizontalOffset();
        indexState = conveyor.getWillBeIndexedState();
        hoodPos = shooter.getHoodPos();
        currentMotorSpeed = shooter.getShooterVelocity();
        currentIndexSpeed = conveyor.getIndexRPM();

        if (prioritizedShootButton.get() && isBallReady() && limelightHasTarget) {

            isHoodSet = setHood(shooter, limelightDistanceEstimate, hoodPos);
            isSpooled = spool(shooter, limelightDistanceEstimate, currentMotorSpeed, hoodPos);
            isDrivenAndAligned = driveAndAlign(drivetrain,
                smoothedHorizontalOffset,
                (indexState == teamColor),
                limelightDistanceEstimate);

            if (isDrivenAndAligned && isHoodSet && isSpooled) {
                fire();
            }
        }
        
        if (!limelightHasTarget) {
            alignPID.reset();
            movePID.reset();
            drivetrain.setBothMotorPower(0);
        }
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();

        prioritizedShootButton = null;
        prioritizedShootButton.destroy();

        shootDelayTimer.stop();

        alignPID.reset();
        movePID.reset();
        alignPID.close();
        movePID.close();
        super.end(false);
    }

    public boolean isFinished() {
        return false;
    }
}
