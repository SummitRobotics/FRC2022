package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import java.lang.Math;

/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {
    // Variables for various things, names are pretty explanitory
    private Shooter shooter;
    private Conveyor conveyor;
    private ConveyorState teamClrIsBlue;
    private Drivetrain drivetrain;
    private NetworkTable hoodTable;
    private NetworkTable speedTable;
    private double distance;
    private NetworkTableEntry speed;
    private NetworkTableEntry hood;
    private boolean hoodAngle;
    private double motorPower;
    private double motorSpeed;
    private NetworkTable limTable;
    private ConveyorState indexState;
    private boolean isBallIndexed;
    private ShooterStates motorState;
    private ShooterStates hoodState;
    private ShooterStates drivStates;
    private ShooterStates convState;
    private double horizontalOffset;
    private static final double ROBOT_RADIUS = 15;

    /**
     * Command for running the shooter in full auto mode.
     *
     * @param shooter The shooter subsystem.
     * @param conveyor The conveyor subsystem.
     * @param drivetrain The drivetrain subsystem.
     * @param limelight The limelight device.
     */
    public FullAutoShooterAssembly(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        Lemonlight limelight) {

        this.shooter = shooter;
        this.conveyor = conveyor;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    // constants
    private static final double TARGET_MOTOR_SPEED = 0;
    
    /**
     * Enum for setting states.
     */
    public enum ShooterStates {
        IDLE,
        REVVING,
        READY,
        MISALIGNED, 
        UNKNOWN
    }

    // devices
    private Lemonlight limelight = new Lemonlight("Shooter");

    /**
     * Initializing variables.
     */
    @Override
    public void initialize() {
        hoodTable = NetworkTableInstance.getDefault().getTable("Hood");
        speedTable = NetworkTableInstance.getDefault().getTable("RPM");
        shooter.stop();
        if (DriverStation.getAlliance().toString() == "kRed") {
            teamClrIsBlue = ConveyorState.RED;
        } else {
            teamClrIsBlue = ConveyorState.BLUE;
        }
        motorState = ShooterStates.IDLE;
        hoodState = ShooterStates.UNKNOWN;
        drivStates = ShooterStates.IDLE;
        convState = ShooterStates.REVVING;
    }

    /**
     * Returns the team color as a ConveyorState.
     *
     * @return color The team color as a ConveyorState.
     */
    public ConveyorState getTeamColor() {
        if (DriverStation.getAlliance().toString() == "kRed") {
            return ConveyorState.RED;
        } else {
            return ConveyorState.BLUE;
        }
    }

    /**
     * Returns the desired motor speed based on the distance from the target and the hood position.
     *
     * @param distance The distance from the target.
     * @param hoodPos The hood position.
     * @return motorSpeed The desired motor speed
     */
    public double solveMotorSpeed(double distance, boolean hoodPos) {
        // Annoyingly, exponent notation requires importing a math library.
        // So, for simplicity, we do not use it.
        // TODO: Test the shooter and do a cubic regression to find the right formula.
        if (hoodPos) {
            return 0 * distance * distance * distance
                + 0 * distance * distance
                + 0 * distance
                + 0;
        } else {
            return 0 * distance * distance * distance
                + 0 * distance * distance
                + 0 * distance
                + 0;
        }
    }

    public void fire() {
        conveyor.setIndexMotorPower(Conveyor.INDEX_MOTOR_POWER);
    }

    /**
     * Returns whether or not there is a ball ready to be fired.
     *
     * @return Whether or not there is a ball ready to be fired.
     */
    public boolean isBallReady() {
        if (indexState != ConveyorState.NONE
            && isBallIndexed) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Move the robot to the target.
     *
     * @param smoothedHorizontalOffset The smoothed horizontal offset
     */
    public void driveToTarget(double smoothedHorizontalOffset) {
        if (smoothedHorizontalOffset > Shooter.TARGET_HORIZONTAL_ACCURACY) {

            CommandScheduler.getInstance().schedule(
                new TurnByEncoder(-Drivetrain.TURN_DEGREES_PER_CYCLE, drivetrain));

        } else if (smoothedHorizontalOffset < -Shooter.TARGET_HORIZONTAL_ACCURACY) {

            CommandScheduler.getInstance().schedule(
                new TurnByEncoder(Drivetrain.TURN_DEGREES_PER_CYCLE, drivetrain));

        } else {

            CommandScheduler.getInstance().schedule(new EncoderDrive(
                Drivetrain.MOVE_FORWARD_PER_CYCLE, Drivetrain.MOVE_FORWARD_PER_CYCLE, drivetrain));
        }
    }

    /**
     * Aims the shooter.
     *
     * @param smoothedHorizontalOffset The smoothed horizontal offset
     * @param teamColor The team color
     */
    public void aim(double smoothedHorizontalOffset, ConveyorState teamColor) {
        if (indexState == teamColor) {

            if (smoothedHorizontalOffset > Shooter.TARGET_HORIZONTAL_ACCURACY) {

                CommandScheduler.getInstance().schedule(
                    new TurnByEncoder(-Drivetrain.TURN_DEGREES_PER_CYCLE, drivetrain));

            } else if (smoothedHorizontalOffset
                < -Shooter.TARGET_HORIZONTAL_ACCURACY) {

                CommandScheduler.getInstance().schedule(
                    new TurnByEncoder(Drivetrain.TURN_DEGREES_PER_CYCLE, drivetrain));

            } else if (indexState != teamColor) {

                if (smoothedHorizontalOffset
                    > Shooter.TARGET_HORIZONTAL_ACCURACY
                    + Shooter.TARGET_WRONG_COLOR_MISS) {

                    CommandScheduler.getInstance().schedule(
                        new TurnByEncoder(-Drivetrain.TURN_DEGREES_PER_CYCLE, drivetrain));

                } else if (smoothedHorizontalOffset
                    < -Shooter.TARGET_HORIZONTAL_ACCURACY
                    + Shooter.TARGET_WRONG_COLOR_MISS) {

                    CommandScheduler.getInstance().schedule(
                        new TurnByEncoder(Drivetrain.TURN_DEGREES_PER_CYCLE, drivetrain));

                }
            }
        }
    }

    @Override
    public void execute() { 
        // Checking Variable       
        motorSpeed = shooter.getShooterVelocity();
        distance = limelight.getLimelightDistanceEstimateIN();
        distance = Math.round(distance);
        hood = hoodTable.getEntry(Double.toString(distance));
        speed = speedTable.getEntry(Double.toString(distance));
        hoodAngle = hood.getBoolean(false);
        motorPower = speed.getDouble(0);
        indexState = conveyor.getWillBeIndexedState();
        isBallIndexed = conveyor.getIsBallIndexed();

        //Checking to make sure we have a ball and it is the same color as our team
        if (isBallIndexed && indexState == teamClrIsBlue && distance > 0) {
            if (motorSpeed != motorPower) {
                shooter.setMotorTargetSpeed(motorPower);
                motorState = ShooterStates.REVVING;
            } else {
                motorState = ShooterStates.READY;
            }
            if (hoodAngle != shooter.getHoodPos()) {
                shooter.setHoodPos(hoodAngle);
                hoodState = ShooterStates.MISALIGNED;
            } else {
                hoodState = ShooterStates.READY;
            }
            if (motorState == ShooterStates.READY && hoodState == ShooterStates.READY) {
                horizontalOffset = limelight.getSmoothedHorizontalOffset();
                if (Math.round(horizontalOffset) != 0) {
                    double radians = Math.PI / 180;
                    double dst = ROBOT_RADIUS * radians;
                    if (drivStates == ShooterStates.IDLE) {
                        drivetrain.stop();
                        drivetrain.zeroDistance();
                        drivetrain.setLeftMotorTarget(drivetrain.distToEncoder(dst));
                        drivetrain.setRightMotorTarget(drivetrain.distToEncoder(-dst));
                        drivStates = ShooterStates.MISALIGNED;
                    }
                    
                } else {
                    drivetrain.stop();
                    drivStates = ShooterStates.IDLE;
                    if (convState == ShooterStates.IDLE) {
                        conveyor.setIndexEncoder(0);
                        conveyor.setIndexMotorPower(.5);
                        convState = ShooterStates.REVVING;
                         
                    } else {
                        if (conveyor.getIndexEncoderPosition() > 2) {
                            conveyor.setIndexMotorPower(0);
                            convState = ShooterStates.IDLE;
                        }
                    }
                }
            }
        }
    }
}
