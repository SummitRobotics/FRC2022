package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.lists.AxisPriorities;


/**
 * Command for running the shooter in full auto mode.
 */
public class SemiAutoShooterAssembly extends FullAutoShooterAssembly {

    private enum TargetState {
        LOOKING_FOR_TARGET,
        NOT_LOOKING_FOR_TARGET,
    }

    // subsystems
    private Shooter shooter;
    private Conveyor conveyor;
    private Drivetrain drivetrain;

    // OI
    OIButton shootButton;
    OIButton.PrioritizedButton prioritizedShootButton;

    // tracker variables
    private ConveyorState indexState;
    private boolean isBallIndexed;
    private TargetState targetState;
    private double motorSpeed;
    private NetworkTable limTable;

    // constants
    private final double TARGET_MOTOR_SPEED = 0;

    // devices
    private Lemonlight limelight;
    
    /**
     * Command for running the shooter in full auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     * @param drivetrain The drivetrain subsystem.
     * @param shootButton The button to enable the shooter.
     */
    public SemiAutoShooterAssembly(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        OIButton shootButton) {
        super(shooter, conveyor, drivetrain);
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shootButton = shootButton;

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        shooter.stop();

        prioritizedShootButton = shootButton.prioritize(AxisPriorities.DEFAULT);
        indexState = conveyor.getWillBeIndexedState();
        targetState = TargetState.NOT_LOOKING_FOR_TARGET;
        motorSpeed = shooter.getShooterVelocity();
    }

    @Override
    public void execute() {
        // Tracker variables to prevent measurements from changing mid-cycle
        indexState = conveyor.getWillBeIndexedState();
        isBallIndexed = conveyor.getIsBallIndexed();
        motorSpeed = shooter.getShooterVelocity();
        limTable = NetworkTableInstance.getDefault().getTable("limelight");

        if (prioritizedShootButton.get()
            && indexState != ConveyorState.NONE
            && isBallIndexed) {

            if (motorSpeed < TARGET_MOTOR_SPEED) {
                if limTable.get
            } else {
                shooter.setMotorTargetSpeed(TARGET_MOTOR_SPEED);
            }
        }
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();

        prioritizedShootButton = null;
        prioritizedShootButton.destroy();
    }
}
