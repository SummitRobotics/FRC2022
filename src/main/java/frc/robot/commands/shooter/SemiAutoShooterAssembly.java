package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.devices.Lemonlight;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.lists.AxisPriorities;


/**
 * Command for running the shooter in full auto mode.
 */
public class SemiAutoShooterAssembly extends FullAutoShooterAssembly {

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
    private double motorSpeed;
    private double limelightDistanceEstimate;
    private boolean limelightHasTarget;


    // constants
    private static final double
        TARGET_MOTOR_SPEED = 0,
        TURN_DEGREES_PER_CYCLE = 1.0,
        MOVE_FORWARD_PER_CYCLE = 1.0,
        SHOOTER_RANGE = 10.0,
        TARGET_HORIZONTAL_ACCURACY = 3;


    // devices
    private Lemonlight limelight;
    
    /**
     * Command for running the shooter in full auto mode.
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
        this.shootButton = shootButton;

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        shooter.stop();

        prioritizedShootButton = shootButton.prioritize(AxisPriorities.DEFAULT);
        
    }

    @Override
    public void execute() {
        // Tracker variables to prevent measurements from changing mid-cycle
        indexState = conveyor.getWillBeIndexedState();
        isBallIndexed = conveyor.getIsBallIndexed();
        motorSpeed = shooter.getShooterVelocity();
        limelightDistanceEstimate = limelight.getLimelightDistanceEstimateIN();
        limelightHasTarget = limelight.hasTarget();


        if (prioritizedShootButton.get()
            && indexState != ConveyorState.NONE
            && isBallIndexed) {

            if (motorSpeed < TARGET_MOTOR_SPEED) {

                if (limelightHasTarget) {

                    if (limelightDistanceEstimate < SHOOTER_RANGE) {

                        if (indexState == ConveyorState.RED) {

                            // Insert further logic here

                        } else if (indexState == ConveyorState.BLUE) {

                            // Insert further logic here

                        }

                    } else {

                        CommandScheduler.getInstance().schedule(new EncoderDrive(
                            MOVE_FORWARD_PER_CYCLE, MOVE_FORWARD_PER_CYCLE, drivetrain));
                    }

                } else {

                    CommandScheduler.getInstance().schedule(
                        new TurnByEncoder(TURN_DEGREES_PER_CYCLE, drivetrain));
                }

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
