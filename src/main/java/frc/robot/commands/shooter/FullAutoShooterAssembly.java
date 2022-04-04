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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.PIDValues;

/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {
    // subsystems
    protected Shooter shooter;
    protected Conveyor conveyor;
    protected Drivetrain drivetrain;
    
    // PID controllers
    protected PIDController movePID;
    protected PIDController alignPID;

    // tracker variables
    protected ConveyorState indexState;
    protected double limelightDistanceEstimate;
    protected boolean limelightHasTarget;
    protected ConveyorState teamColor;
    protected double smoothedHorizontalOffset;
    protected boolean hoodPos;
    protected boolean isSpooled;
    protected boolean isHoodSet;
    protected double currentMotorSpeed;
    protected double currentIndexSpeed;
    protected boolean isDrivenAndAligned;
    boolean properlyAligned;

    //constants
    protected static final double
        MAX_SHOOTER_RANGE = 180,
        MIN_SHOOTER_RANGE = 45,
        OK_TO_MOVE_OFSET = 5,
        HOOD_UP_RANGE = 125,
        RANGE_OVERLAP = 4,
        TARGET_HORIZONTAL_ACCURACY = 5,
        TARGET_WRONG_COLOR_MISS = 30,
        TARGET_MOTOR_SPEED_ACCURACY = 75,
        IDEAL_SHOOTING_DISTANCE = 60,
        SHOOTER_IDLE_SPEED = 1700;

    // devices
    protected Lemonlight limelight;

    protected boolean hasRecordedLimelightDistance = false;

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

        this.alignPID = new PIDController(PIDValues.ALIGN_P, PIDValues.ALIGN_I, PIDValues.ALIGN_D);
        this.movePID = new PIDController(PIDValues.MOVE_P, PIDValues.MOVE_I, PIDValues.MOVE_D);

        // TODO - Set these, including the constants
        alignPID.setTolerance(TARGET_HORIZONTAL_ACCURACY, 9999999);
        movePID.setTolerance(2, 9999999);
        alignPID.setSetpoint(0);
        movePID.setSetpoint(IDEAL_SHOOTING_DISTANCE);

        addRequirements(shooter, drivetrain);
    }

    /**
     * Returns the team color as a ConveyorState.
     *
     * @return color The team color as a ConveyorState.
     */
    public ConveyorState getTeamColor() {
        if (DriverStation.getAlliance().toString() == "Red") {
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
        // TODO - Test the shooter and do a regression to find the right formula.
        // Add higher order terms if necessary.
        if (hoodPos) {
            System.out.println("reg far");
            return
                (-0.00330099 * distance * distance * distance) +
                (1.44904 * distance * distance) +
                (-205.058 * distance) +
                11204.6;
        } else {
            System.out.println("reg close");
            return 
                (-0.00000243868 * distance * distance * distance * distance * distance) +
                (0.000923236 * distance * distance * distance * distance) +
                (-0.131588 * distance * distance * distance) +
                (8.76388 * distance * distance) +
                (-266.098 * distance) +
                4428.38;
        }
    }

    /**
     * Spins the index motor 360 degrees, moving a ball into the shooter.
     *
     * @param shooter The shooter subsystem
     */
    public void fire(Shooter shooter) {
        //System.out.println("FFFFFFFFFFFIIIIIIIIIRRRRRRRRRRREEEEEEEEEEEEEEEE");
        shooter.setState(Shooter.States.READY_TO_FIRE);
        properlyAligned = false;

    }

    /**
     * Returns whether or not there is a ball ready to be fired.
     *
     * @return Whether or not there is a ball ready to be fired.
     */
    public boolean isBallReady() {
        return (indexState != ConveyorState.NONE && conveyor.isBallIndexed());
    }

    /**
     * Method to drive into range and align with the target. We only run this if we have a target.
     *
     * @param drivetrain The drivetrain subsystem
     * @param horizontalOffset The angle between us and the target, as reported by the limelight
     * @param isAccurate Whether we are shooting accurately or intentionally missing
     * @param limelightDistanceEstimate The limelight's reported distance estimate from the target
     * @return Whether or not we are aligned and in range
     */
    public boolean driveAndAlign(Drivetrain drivetrain,
        double horizontalOffset,
        boolean isAccurate,
        double limelightDistanceEstimate,
        double setpoint) {
        //sets if align target based on ball color
        if (movePID.atSetpoint()) {
            alignPID.setSetpoint(Math.toDegrees(Math.atan((12 / limelightDistanceEstimate))));
            properlyAligned = true;
        } else if (!properlyAligned) {
            alignPID.setSetpoint(0);
        }

        double leftPower = 0;
        double rightPower = 0;

        //align to target
        double alignPower = alignPID.calculate(horizontalOffset);

        leftPower += -alignPower;
        rightPower += alignPower;


        //move towards target
        if (Functions.isWithin(horizontalOffset, alignPID.getSetpoint(), OK_TO_MOVE_OFSET)) {
            //records current distance to be heald with pid, clamped to the max and min range of the shooter
            if (!hasRecordedLimelightDistance) {
                movePID.setSetpoint(Functions.clampDouble(limelightDistanceEstimate, MAX_SHOOTER_RANGE - 5, MIN_SHOOTER_RANGE));
                hasRecordedLimelightDistance = true;
                
            }
            if (!isAccurate) {
                movePID.setSetpoint(0);
            } else {
                //alignPID.setSetpoint(0);
                movePID.setSetpoint(setpoint);
            }    
            double movePower = movePID.calculate(limelightDistanceEstimate);
            leftPower -= movePower;
            rightPower -= movePower;

        } else {
            hasRecordedLimelightDistance = false;
        }

        //sets drivetrain powers
        drivetrain.setLeftMotorPower(leftPower);
        drivetrain.setRightMotorPower(rightPower);
        System.out.println("Is aligned " + alignPID.atSetpoint() + " is arrived " + movePID.atSetpoint());
        return alignPID.atSetpoint() && movePID.atSetpoint() && properlyAligned;
    }

    /**
     * Method to spin the drivetrain to locate a target.
     *
     * @param drivetrain The drivetrain subsystem
     */
    public void findTarget(Drivetrain drivetrain) {
        drivetrain.setLeftMotorPower(0.25);
        drivetrain.setRightMotorPower(-0.25);
    }

    /**
     * Method to spool the flywheel to the correct speed.
     *
     * @param shooter The shooter subsystem
     * @param limelightDistanceEstimate The limelight's reported distance estimate from the target
     * @param currentMotorSpeed The current motor speed
     * @param hoodPos The current hood position
     * @param isAligned Whether or not we are aligned
     * @return Whether or not the motor was already spooled
     */
    public boolean spool(Shooter shooter,
        double limelightDistanceEstimate,
        double currentMotorSpeed,
        boolean hoodPos,
        boolean isAligned) {

        double targetMotorSpeed = solveMotorSpeed(limelightDistanceEstimate, hoodPos);

        if (isAligned) {
            if (!Functions.isWithin(currentMotorSpeed, targetMotorSpeed, TARGET_MOTOR_SPEED_ACCURACY)) {
    
                shooter.setMotorTargetSpeed(targetMotorSpeed);
                return false;
    
            } else {
                return true;
            }
        } else {
            shooter.setMotorTargetSpeed(1500);
            return false;
        }
    }

    /**
     * Method to set the hood position to what it should be.
     *
     * @param shooter The shooter subsystem
     * @param limelightDistanceEstimate The reported distance between the limelight and the target
     * @param hoodPos The hood position
     * @return Whether or not the hood position was correct
     */
    public boolean setHood(Shooter shooter, double limelightDistanceEstimate, boolean hoodPos) {
        if ((limelightDistanceEstimate < HOOD_UP_RANGE - RANGE_OVERLAP && hoodPos != false) || (limelightDistanceEstimate > HOOD_UP_RANGE + RANGE_OVERLAP && hoodPos != true)) {

            shooter.toggleHoodPos();
            return false;

        } else {
            return true;
        }
    }

    @Override
    public void initialize() {
        teamColor = getTeamColor();
        alignPID.reset();
        movePID.reset();
        hasRecordedLimelightDistance = false;
    }
    
    @Override
    public void execute() {
        limelightHasTarget = limelight.hasTarget();
        limelightDistanceEstimate = Lemonlight.getLimelightDistanceEstimateIN(
            Lemonlight.MAIN_MOUNT_HEIGHT,
            Lemonlight.MAIN_MOUNT_ANGLE,
            Lemonlight.MAIN_TARGET_HEIGHT,
            limelight.getVerticalOffset());
        smoothedHorizontalOffset = limelight.getHorizontalOffset();
        indexState = conveyor.getIndexState();
        hoodPos = shooter.getHoodPos();
        currentMotorSpeed = shooter.getShooterRPM();
        isSpooled = false;
        if (limelightHasTarget) {
            //TODO change this back
            isHoodSet = setHood(shooter, limelightDistanceEstimate, hoodPos);
            isDrivenAndAligned = driveAndAlign(drivetrain,
                smoothedHorizontalOffset,
                true,
                limelightDistanceEstimate,
                0);
            if (isHoodSet) {
                isSpooled = spool(shooter, limelightDistanceEstimate,
                currentMotorSpeed, hoodPos, isDrivenAndAligned);
            }

            if (!isDrivenAndAligned) {
                shooter.setState(Shooter.States.DRIVING_AND_ALIGNING);
            } else if (!isHoodSet) {
                shooter.setState(Shooter.States.SETTING_HOOD);
            } else if (!isSpooled) {
                shooter.setState(Shooter.States.SPOOLING);
            } else if (!isBallReady()) {
                shooter.setState(Shooter.States.NO_BALL);
            } else {
                fire(shooter);
            }

        } else {
            properlyAligned = false;            
            shooter.setMotorTargetSpeed(SHOOTER_IDLE_SPEED);
            alignPID.reset();
            movePID.reset();
            findTarget(drivetrain);
            shooter.setState(Shooter.States.NO_TARGET);
            hasRecordedLimelightDistance = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
        shooter.stop();
        
        alignPID.reset();
        movePID.reset();
        alignPID.close();
        movePID.close();
        drivetrain.stop();
        shooter.setState(Shooter.States.NOT_SHOOTING);
        hasRecordedLimelightDistance = false;
    }

    /**
     * Returns whether or not the command should end.
     *
     * @return whether or not the command should end
     */
    public boolean isFinished() {
        System.out.println(!conveyor.doesBallExist());
        return !conveyor.doesBallExist();
    }
}
