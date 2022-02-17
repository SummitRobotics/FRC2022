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

    // subsystems
    private Shooter shooter;
    private Conveyor conveyor;
    private Drivetrain drivetrain;

    // OI
    private OIButton shootButton;
    private OIButton.PrioritizedButton prioritizedShootButton;

    // devices
    private Lemonlight limelight;

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
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shootButton = shootButton;

        alignRightPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        alignWrongPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);

        // TODO - Set these, including the constants
        alignRightPID.setTolerance(Shooter.TARGET_HORIZONTAL_ACCURACY, 1);
        alignWrongPID.setTolerance(Shooter.TARGET_HORIZONTAL_ACCURACY, 1);
        movePID.setTolerance(1, 1);
        alignRightPID.setSetpoint(0);
        alignWrongPID.setSetpoint(Shooter.TARGET_WRONG_COLOR_MISS);
        movePID.setSetpoint(Drivetrain.IDEAL_SHOOTING_DISTANCE);

        addRequirements(shooter, drivetrain, conveyor);
    }

    @Override
    public void initialize() {
        shooter.stop();
        teamColor = getTeamColor();
        prioritizedShootButton = shootButton.prioritize(AxisPriorities.DEFAULT);
        alignRightPID.reset();
        alignWrongPID.reset();
        movePID.reset();

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
            } else if (currentIndexSpeed != 0) {
                conveyor.setIndexMotorPower(0);
            }
        } else if (currentIndexSpeed != 0) {
            conveyor.setIndexMotorPower(0);
        }
        
        if (!limelightHasTarget) {
            alignRightPID.reset();
            alignWrongPID.reset();
            movePID.reset();
            drivetrain.setBothMotorPower(0);
        }
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();

        prioritizedShootButton = null;
        prioritizedShootButton.destroy();

        alignRightPID.reset();
        alignWrongPID.reset();
        movePID.reset();
        alignRightPID.close();
        alignWrongPID.close();
        movePID.close();
    }

    public boolean isFinished() {
        return false;
    }
}
