package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
import java.lang.Math;

/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {
    // Variables for various things, names are pretty explanitory
    private Shooter shooter;
    private Conveyor conveyor;
    private ConveyorState teamClr;
    private Drivetrain drivetrain;
    private NetworkTable hoodTable;
    private NetworkTable speedTable;
    private double distance;
    private NetworkTableEntry speed;
    private NetworkTableEntry hood;
    private boolean hoodAngle;
    private double motorPower;
    private double motorSpeed;
    private ConveyorState indexState;
    private boolean isBallIndexed;
    private ShooterStates motorState;
    private ShooterStates hoodState;
    private ShooterStates drivStates;
    private ShooterStates convState;
    private double horizontalOffset;
    private PIDController alignRightPID;
    private PIDController alignWrongPID;
    private PIDController movePID;

       
    // constants
    private static final double
        ALIGN_P = 0,
        ALIGN_I = 0,
        ALIGN_D = 0,
        MOVE_P = 0,
        MOVE_I = 0,
        MOVE_D = 0,
        ROBOT_RADIUS = 15;
    
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

        alignRightPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        alignWrongPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);

        // TODO - Set these
        alignRightPID.setTolerance(1, 2);
        alignWrongPID.setTolerance(1, 2);
        movePID.setTolerance(1, 2);


        addRequirements(shooter, drivetrain, conveyor);
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
     * Method to drive into the range needed by the shooter.
     *
     * @param drivetrain The drivetrain subsystem
     * @param alignRightPID The PID controller in use for aiming accurately
     * @param alignWrongPID The PID controller in use for aiming inaccurately
     * @param movePID The PID controller in use for moving towards the target
     * @param distance The reported distance between the limelight and the target
     * @param hasTarget Whether or not the limelight has a target
     * @param horizontalOffset The limelight's measurement of the horizontal offset
     * @return Whether or not we were in the right position
     */
    public boolean driveToTarget(
        Drivetrain drivetrain,
        PIDController alignRightPID,
        PIDController alignWrongPID,
        PIDController movePID,
        double distance,
        boolean hasTarget,
        double horizontalOffset) {

        if (alignWithTarget(
            drivetrain,
            alignRightPID,
            alignWrongPID,
            hasTarget,
            horizontalOffset,
            true)) {

            if (distance > Shooter.SHOOTER_RANGE) {
                drivetrain.setBothMotorPower(movePID.calculate(distance));
                return false;
            } else {
                drivetrain.setBothMotorPower(0);
                return true;
            }
        } else {
            return false;
        }
    }

    /**
     * Method to align the drivetrain with the target.
     *
     * @param drivetrain The drivetrain subsystem
     * @param alignRightPID The PID controller in use for aligning directly towards the target
     * @param alignWrongPID The PID controller in use for aligning to the side
     * @param hasTarget Whether or not the limelight has a target
     * @param horizontalOffset The limelight's measurement of the horizontal offset
     * @param isAccurate Whether or not our aim should be off
     * @return Whether or not we were aligned
     */
    public boolean alignWithTarget(Drivetrain drivetrain,
        PIDController alignRightPID,
        PIDController alignWrongPID,
        boolean hasTarget,
        double horizontalOffset,
        boolean isAccurate) {


        if (!Functions.isWithin(horizontalOffset, 0, Shooter.TARGET_HORIZONTAL_ACCURACY)) {
            if (hasTarget) {
                if (isAccurate) {
                    drivetrain.setLeftMotorPower(alignRightPID.calculate(horizontalOffset));
                    drivetrain.setRightMotorPower(-alignRightPID.calculate(horizontalOffset));
                } else {
                    drivetrain.setLeftMotorPower(alignWrongPID.calculate(horizontalOffset));
                    drivetrain.setRightMotorPower(-alignWrongPID.calculate(horizontalOffset));
                }
            } else {
                alignRightPID.reset();
                alignWrongPID.reset();
            }
            return false;

        } else {
            return true;
        }
    }

    /**
     * Method to spin the drivetrain to locate a target.
     *
     * @param drivetrain The drivetrain subsystem
     * @param hasTarget Whether or not the limelight has a target
     */
    public void findTarget(Drivetrain drivetrain, boolean hasTarget) {
        if (!hasTarget) {
            drivetrain.setLeftMotorPower(0.5);
            drivetrain.setRightMotorPower(-0.5);
        } else {
            drivetrain.setBothMotorPower(0);
        }
    }

    /**
     * Initializing variables.
     */
    @Override
    public void initialize() {
        hoodTable = NetworkTableInstance.getDefault().getTable("Hood");
        speedTable = NetworkTableInstance.getDefault().getTable("RPM");
        shooter.stop();
        teamClr = getTeamColor();
        motorState = ShooterStates.IDLE;
        hoodState = ShooterStates.UNKNOWN;
        drivStates = ShooterStates.IDLE;
        convState = ShooterStates.REVVING;
        alignRightPID.setSetpoint(0);
        alignRightPID.reset();
        alignWrongPID.setSetpoint(0);
        alignWrongPID.reset();
        movePID.setSetpoint(0);
        movePID.reset();

    }

    /*@Override
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
        if (isBallIndexed && indexState == teamClr && distance > 0) {
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
    }*/

    @Override
    public void end(boolean interrupted) {
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
