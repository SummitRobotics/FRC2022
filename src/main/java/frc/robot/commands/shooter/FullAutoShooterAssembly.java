package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;

/**
 * Command for running the shooter in full auto mode.
 */
public class FullAutoShooterAssembly extends CommandBase {
    // subsystems
    private Shooter shooter;
    private Conveyor conveyor;
    private Drivetrain drivetrain;
    
    // PID controllers
    protected PIDController alignWrongPID;
    protected PIDController movePID;
    protected PIDController alignRightPID;

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
       
    // PID values
    protected static final double
        ALIGN_P = 0,
        ALIGN_I = 0,
        ALIGN_D = 0,
        MOVE_P = 0,
        MOVE_I = 0,
        MOVE_D = 0;

    // devices
    private Lemonlight limelight;

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

        // TODO - Set these, including the constants
        alignRightPID.setTolerance(Shooter.TARGET_HORIZONTAL_ACCURACY, 1);
        alignWrongPID.setTolerance(Shooter.TARGET_HORIZONTAL_ACCURACY, 1);
        movePID.setTolerance(1, 1);
        alignRightPID.setSetpoint(0);
        alignWrongPID.setSetpoint(Shooter.TARGET_WRONG_COLOR_MISS);
        movePID.setSetpoint(Drivetrain.IDEAL_SHOOTING_DISTANCE);

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
        // TODO - Test the shooter and do a regression to find the right formula.
        // Add higher order terms if necessary.
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
            && conveyor.getIsBallIndexed()) {
            return true;
        } else {
            return false;
        }
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
        double limelightDistanceEstimate) {
        
        if (limelightDistanceEstimate > Shooter.SHOOTER_RANGE) {
            if (!Functions.isWithin(horizontalOffset, 0, Shooter.TARGET_HORIZONTAL_ACCURACY)) {

                drivetrain.setLeftMotorPower(alignRightPID.calculate(horizontalOffset));
                drivetrain.setRightMotorPower(-alignRightPID.calculate(horizontalOffset));
                return false;
            } else {
                drivetrain.setBothMotorPower(movePID.calculate(limelightDistanceEstimate));
                return false;
            }
        } else {
            if (isAccurate) {
                if (!Functions.isWithin(horizontalOffset, 0, Shooter.TARGET_HORIZONTAL_ACCURACY)) {
                    drivetrain.setLeftMotorPower(alignRightPID.calculate(horizontalOffset));
                    drivetrain.setRightMotorPower(-alignRightPID.calculate(horizontalOffset));
                    return false;

                } else {
                    drivetrain.setBothMotorPower(0);
                    return true;
                }
                
            } else {
                if (!Functions.isWithin(horizontalOffset,
                    Shooter.TARGET_WRONG_COLOR_MISS,
                    Shooter.TARGET_HORIZONTAL_ACCURACY)) {

                    drivetrain.setLeftMotorPower(alignWrongPID.calculate(horizontalOffset));
                    drivetrain.setRightMotorPower(-alignWrongPID.calculate(horizontalOffset));
                    return false;
                    
                } else {
                    drivetrain.setBothMotorPower(0);
                    return true;
                }
            }
        }
    }

    /**
     * Method to spin the drivetrain to locate a target.
     *
     * @param drivetrain The drivetrain subsystem
     */
    public void findTarget(Drivetrain drivetrain) {
        drivetrain.setLeftMotorPower(0.5);
        drivetrain.setRightMotorPower(-0.5);
    }

    /**
     * Method to spool the flywheel to the correct speed.
     *
     * @param shooter The shooter subsystem
     * @param limelightDistanceEstimate The limelight's reported distance estimate from the target
     * @param currentMotorSpeed The current motor speed
     * @param hoodPos The current hood position
     * @return Whether or not the motor was already spooled
     */
    public boolean spool(Shooter shooter,
        double limelightDistanceEstimate,
        double currentMotorSpeed,
        boolean hoodPos) {

        double targetMotorSpeed = solveMotorSpeed(limelightDistanceEstimate, hoodPos);

        if (!Functions.isWithin(currentMotorSpeed,
            targetMotorSpeed,
            Shooter.TARGET_MOTOR_SPEED_ACCURACY)) {

            shooter.setMotorTargetSpeed(targetMotorSpeed);
            return false;

        } else {
            return true;
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
        if (limelightDistanceEstimate < Shooter.SHOOTER_RANGE) {
            if ((limelightDistanceEstimate < Shooter.HOOD_UP_RANGE - Shooter.RANGE_OVERLAP
                && hoodPos == false)
                || (limelightDistanceEstimate > Shooter.HOOD_UP_RANGE + Shooter.RANGE_OVERLAP
                && hoodPos == true)) {

                shooter.toggleHoodPos();
                return false;

            } else {
                return true;
            }

        } else {
            return false;
        }
    }

    /**
     * Initializing variables.
     */
    @Override
    public void initialize() {
        shooter.stop();
        teamColor = getTeamColor();
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

        if (isBallReady() && limelightHasTarget) {

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
            findTarget(drivetrain);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        
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
