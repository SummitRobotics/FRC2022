package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.devices.Lemonlight;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
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
    OIButton shootButton;
    OIButton.PrioritizedButton prioritizedShootButton;

    // tracker variables
    private ConveyorState indexState;
    private boolean isBallIndexed;
    private double motorSpeed;
    private double limelightDistanceEstimate;
    private boolean limelightHasTarget;
    private ConveyorState teamColor;
    private double smoothedHorizontalOffset;
    private double targetMotorSpeed;

    // devices
    private Lemonlight limelight;

    // PID controllers
    private PIDController alignRightPID;
    private PIDController alignWrongPID;
    private PIDController movePID;
    
    // PID values
    // TODO - Set these
    private static final double
        ALIGN_P = 0,
        ALIGN_I = 0,
        ALIGN_D = 0,
        MOVE_P = 0,
        MOVE_I = 0,
        MOVE_D = 0;

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
        this.shootButton = shootButton;

        alignRightPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        alignWrongPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);
        movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);

        // TODO - Set these
        alignRightPID.setTolerance(1, 2);
        alignWrongPID.setTolerance(1, 2);
        movePID.setTolerance(1, 2);

        addRequirements(shooter, conveyor, drivetrain);
    }

    @Override
    public void initialize() {
        shooter.stop();
        teamColor = getTeamColor();
        prioritizedShootButton = shootButton.prioritize(AxisPriorities.DEFAULT);
        
    }

    /*@Override
    public void execute() {
        // Tracker variables to prevent measurements from changing mid-cycle
        indexState = conveyor.getWillBeIndexedState();
        isBallIndexed = conveyor.getIsBallIndexed();
        motorSpeed = shooter.getShooterVelocity();
        limelightDistanceEstimate = limelight.getLimelightDistanceEstimateIN();
        limelightHasTarget = limelight.hasTarget();
        smoothedHorizontalOffset = limelight.getSmoothedHorizontalOffset();


        if (prioritizedShootButton.get()
            && indexState != ConveyorState.NONE
            && isBallIndexed) {

            if (limelightHasTarget) {

                if (limelightDistanceEstimate < Shooter.SHOOTER_RANGE) {

                    if (limelightDistanceEstimate < Shooter.HOOD_UP_RANGE) {
                        shooter.setHoodPos(true);
                        targetMotorSpeed = solveMotorSpeed(limelightDistanceEstimate, true);
                    } else {
                        shooter.setHoodPos(false);
                        targetMotorSpeed = solveMotorSpeed(limelightDistanceEstimate, false);
                    }

                    if (Functions.isWithin(
                        motorSpeed, targetMotorSpeed, Shooter.TARGET_MOTOR_SPEED_ACCURACY)) {

                        if (indexState == teamColor) {

                            if (smoothedHorizontalOffset > Shooter.TARGET_HORIZONTAL_ACCURACY) {

                                CommandScheduler.getInstance().schedule(
                                    new TurnByEncoder(-Drivetrain.TURN_DEGREES_PER_CYCLE,
                                    drivetrain));

                            } else if (smoothedHorizontalOffset
                                < -Shooter.TARGET_HORIZONTAL_ACCURACY) {

                                CommandScheduler.getInstance().schedule(
                                    new TurnByEncoder(Drivetrain.TURN_DEGREES_PER_CYCLE,
                                    drivetrain));

                            } else {
                                // Everything seems in order, so trigger the index motor.
                                fire();
                            }

                        } else if (indexState != teamColor) {

                            if (smoothedHorizontalOffset
                                > Shooter.TARGET_HORIZONTAL_ACCURACY
                                + Shooter.TARGET_WRONG_COLOR_MISS) {

                                CommandScheduler.getInstance().schedule(
                                    new TurnByEncoder(-Drivetrain.TURN_DEGREES_PER_CYCLE,
                                    drivetrain));

                            } else if (smoothedHorizontalOffset
                                < -Shooter.TARGET_HORIZONTAL_ACCURACY
                                + Shooter.TARGET_WRONG_COLOR_MISS) {

                                CommandScheduler.getInstance().schedule(
                                    new TurnByEncoder(Drivetrain.TURN_DEGREES_PER_CYCLE,
                                    drivetrain));

                            } else {
                                // Everything seems in order, so trigger the index motor.
                                fire();
                            }
                        }

                    } else {
                        shooter.setMotorTargetSpeed(targetMotorSpeed);
                    }

                } else {
                    driveToTarget(smoothedHorizontalOffset);
                }

            } else {

                CommandScheduler.getInstance().schedule(
                    new TurnByEncoder(Drivetrain.TURN_DEGREES_PER_CYCLE, drivetrain));
            }
        }
    }*/

    @Override
    public void execute() {
        limelightHasTarget = limelight.hasTarget();
        limelightDistanceEstimate = limelight.getLimelightDistanceEstimateIN();
        smoothedHorizontalOffset = limelight.getSmoothedHorizontalOffset();
        indexState = conveyor.getWillBeIndexedState();

        if (prioritizedShootButton.get() && isBallReady() && limelightHasTarget) {

            if (driveToTarget(
                    drivetrain,
                    alignRightPID,
                    alignWrongPID,
                    movePID,
                    limelightDistanceEstimate,
                    limelightHasTarget,
                    smoothedHorizontalOffset)
                && alignWithTarget(
                    drivetrain,
                    alignRightPID,
                    alignWrongPID,
                    limelightHasTarget,
                    smoothedHorizontalOffset,
                    (indexState == teamColor))) {
                
                // shoot the ball
                
            }

            
        }
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();

        prioritizedShootButton = null;
        prioritizedShootButton.destroy();
    }

    public boolean isFinished() {
        return false;
    }
}
