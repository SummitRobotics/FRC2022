package frc.robot.commands.shooter;

import frc.robot.devices.Lemonlight;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;


/**
 * Command for running the shooter in semi auto mode.
 */
public class SemiAutoShooterAssembly extends FullAutoShooterAssembly {

    // OI
    private OIButton shootButton;
    private OIButton.PrioritizedButton prioritizedShootButton;
    private OIAxis controlAxis;
    private OIAxis.PrioritizedAxis prioritizedControlAxis;
    private final int axisPriority = 5;

    /**
     * Command for running the shooter in semi auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     * @param drivetrain The drivetrain subsystem.
     * @param limelight The limelight device.
     * @param shootButton The button to enable the shooter.
     * @param controlAxis The joystick used to manually align the robot to find a target.
     */
    public SemiAutoShooterAssembly(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        Lemonlight limelight,
        OIButton shootButton,
        OIAxis controlAxis) {

        super(shooter, conveyor, drivetrain, limelight);
        super.initialize();
        this.shootButton = shootButton;
        this.controlAxis = controlAxis;

        addRequirements(shooter, drivetrain, conveyor);
    }

    @Override
    public void findTarget(Drivetrain drivetrain) {
        drivetrain.setLeftMotorPower(prioritizedControlAxis.get());
        drivetrain.setRightMotorPower(-prioritizedControlAxis.get());

    }

    @Override
    public void fire() {
        if (prioritizedShootButton.get()) {
            super.fire();
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        
        prioritizedShootButton = shootButton.prioritize(axisPriority);
        prioritizedControlAxis = controlAxis.prioritize(axisPriority);
    }

    @Override
    public void end(final boolean interrupted) {
        super.end(interrupted);

        prioritizedShootButton = null;
        prioritizedShootButton.destroy();
        prioritizedControlAxis.destroy();
    }

}
