package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.devices.Lemonlight;
import frc.robot.oi.inputs.OIAxis;
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
    private OIButton driveButton;
    private OIButton alignButton;
    private OIButton.PrioritizedButton prioritizedShootButton;
    private OIButton.PrioritizedButton prioritizedDriveButton;
    private OIButton.PrioritizedButton prioritizedAlignButton;
    private OIAxis controlAxis;
    private OIAxis.PrioritizedAxis prioritizedControlAxis;
    private final int axisPriority = AxisPriorities.MANUAL_OVERRIDE;
    private Command arcadeDrive;

    /**
     * Command for running the shooter in semi auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     * @param drivetrain The drivetrain subsystem.
     * @param limelight The limelight device.
     * @param shootButton The button to enable the shooter.
     * @param controlAxis The joystick used to manually align the robot to find a target.
     * @param arcadeDrive The arcadeDrive command
     */
    public SemiAutoShooterAssembly(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        Lemonlight limelight,
        OIButton shootButton,
        OIAxis controlAxis,
        OIButton alignButton,
        OIButton driveButton, 
        Command arcadeDrive) {

        super(shooter, conveyor, drivetrain, limelight);
        this.shootButton = shootButton;
        this.controlAxis = controlAxis;
        this.arcadeDrive = arcadeDrive;
        this.alignButton = alignButton;
        this.driveButton = driveButton;

        addRequirements(shooter, drivetrain);
    }

    @Override
    public void findTarget(Drivetrain drivetrain) {
        arcadeDrive.execute();
    }

    @Override
    public boolean driveAndAlign(Drivetrain drivetrain,
        double horizontalOffset,
        boolean isAccurate,
        double limelightDistanceEstimate) {
        boolean x;
        if (prioritizedDriveButton.get()) {
            x = super.driveAndAlign(drivetrain, limelight.getHorizontalOffset(), true, limelight.getLimelightDistanceEstimateIN(limelight.MAIN_MOUNT_HEIGHT, limelight.MAIN_MOUNT_ANGLE, limelight.MAIN_TARGET_HEIGHT, limelight.getVerticalOffset()));
        } else if (prioritizedAlignButton.get()) {
            x = super.driveAndAlign(drivetrain, limelight.getHorizontalOffset(), true, super.IDEAL_SHOOTING_DISTANCE);
        } else {
            x = true;
        }
        return x;
    }
    
    @Override
    public void fire(Shooter shooter) {
        //System.out.println(prioritizedShootButton.get());
        if (prioritizedShootButton.get()) {
            //System.out.println("semi fire");
            super.fire(shooter);
        } else {
            shooter.setState(Shooter.States.NOT_SHOOTING);
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        
        prioritizedShootButton = shootButton.prioritize(axisPriority);
        prioritizedControlAxis = controlAxis.prioritize(axisPriority);
        prioritizedAlignButton = alignButton.prioritize(axisPriority);
        prioritizedDriveButton = alignButton.prioritize(axisPriority);
    }

    @Override
    public void execute() {
        super.execute();
        driveAndAlign(drivetrain, 0, true, 0);
        fire(shooter);
    }

    @Override
    public void end(final boolean interrupted) {
        super.end(interrupted);

        prioritizedShootButton.destroy();
        prioritizedControlAxis.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
