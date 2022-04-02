package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.Functions;
import java.util.ArrayList;

// TODO: 
// continue moving if loses sight
// end if conveyor gets new ball

/**
 * Full auto intake mode.
 */
public class FullAutoIntakeDrive extends CommandBase {

    private static final double 
        DISTANCE_FROM_BALL = 0,
        BACK_UP_THRESHOLD = 14,
        BACK_UP_DISTANCE = 24;

    private ConveyorState lastState;
    private ConveyorState thisState;
    private boolean changed;
    // subsystems
    private Drivetrain drivetrain;
    private String teamColor;
    // devices
    private Lemonlight limelight;
    // PID controllers
    private PIDController movePID;
    private PIDController alignPID;
    private double oldDistance;
    // tracker variables
    private double limelightDistanceEstimate;
    private double horizontalOffset;
    private Conveyor conveyor;
    private Intake intake;
    private boolean isBackingUp;

    /**
     * Constructor.
     *
     * @param drivetrain The drivetrain subsystem
     * @param limelight The ball detection limelight
     * @param conveyor for states to check if ball was intaked (what is the past tense verb for intake? intook?)
     * @param intake intake to control speed when picking up balls
     */

    public FullAutoIntakeDrive(Drivetrain drivetrain,
        Lemonlight limelight, Conveyor conveyor, Intake intake) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.conveyor = conveyor;
        this.movePID = new PIDController(0.02, 0, 0);
        this.alignPID = new PIDController(0.005, 0, 0);
        this.intake = intake;
        // TODO - set these
        movePID.setTolerance(1, 1);
        movePID.setSetpoint(DISTANCE_FROM_BALL);
        alignPID.setTolerance(1, 1);
        alignPID.setSetpoint(0);

        addRequirements(drivetrain, intake);
        teamColor = DriverStation.getAlliance().toString();
        isBackingUp = false;
    }

    @Override
    public void initialize() {
        teamColor = DriverStation.getAlliance().toString();
        movePID.reset();
        alignPID.reset();
        oldDistance = 999;
        changed = false;
        lastState = conveyor.getBeltState();
    }

    @Override
    public void execute() {
        double distTest;
        thisState = conveyor.getBeltState();
        double color = -100;
        horizontalOffset = 0;
        ArrayList<double[]> limelightData = limelight.getCustomVisionDataReadable();
        limelightDistanceEstimate = 999;
        if (limelightData.size() < 1) {
            System.out.println("NO BALL");
            //drivetrain.stop();
            return;
        }
        for (double[] eachLimelight : limelightData) {
            if ((eachLimelight[0] == 0.0 && teamColor == "Red") || (eachLimelight[0] == 1.0 && teamColor == "Blue")) {
                distTest = Lemonlight.getLimelightDistanceEstimateIN(
                    Lemonlight.BALL_MOUNT_HEIGHT,
                    Lemonlight.BALL_MOUNT_ANGLE,
                    Lemonlight.BALL_TARGET_HEIGHT,
                    -eachLimelight[2]) + 19;
                if (limelightDistanceEstimate > distTest) {
                    color = eachLimelight[0];
                    limelightDistanceEstimate = distTest;
                    horizontalOffset = eachLimelight[1];
                }
            }

        }

        System.out.println("DIST: " + limelightDistanceEstimate + " offset: " + horizontalOffset + " COLOR: " + color);

        if (limelightDistanceEstimate < 999) {

            // if (limelightDistanceEstimate < BACK_UP_THRESHOLD
            //     && !Functions.isWithin(horizontalOffset, 0, 5)) {
                
            //     isBackingUp = true;
            //     movePID.setSetpoint(BACK_UP_DISTANCE);
            //     alignPID.setSetpoint(horizontalOffset);

            // }
            
            // if (isBackingUp) {
            //     if (movePID.atSetpoint()) {
            //         alignPID.setSetpoint(0);
            //         if (alignPID.atSetpoint()) {
            //             movePID.setSetpoint(0);
            //             isBackingUp = false;
            //         }
            //     }
            // }

            double alignPower = alignPID.calculate(horizontalOffset);
            oldDistance = limelightDistanceEstimate;
            double movePower = -Functions.clampDouble(movePID.calculate(limelightDistanceEstimate), 0.5, -0.5);
            drivetrain.setLeftMotorPower(movePower - alignPower);
            drivetrain.setRightMotorPower(movePower + alignPower);
            intake.setIntakeEncoder(0);
        }
        if (lastState != thisState) {
            lastState = thisState;
            if (thisState != ConveyorState.NONE) {
                changed = true;
            }
            //drivetrain.stop();
        }
        }   

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return (conveyor.getIndexState() != ConveyorState.NONE && conveyor.getBeltState() != ConveyorState.NONE) || changed;
    }
}
