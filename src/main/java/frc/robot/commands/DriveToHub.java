package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.PIDValues;

/**
 * Full auto intake mode.
 */

public class DriveToHub extends CommandBase {

    // TODO - set this
    private static final double DISTANCE_FROM_HUB = 0;
    private final Timer timer = new Timer();
    // subsystems
    private Drivetrain drivetrain;

    // devices
    private Lemonlight limelight;

    // PID controllers
    private PIDController movePID;
    private PIDController alignPID;

    // tracker variables
    private double limelightDistanceEstimate;
    private boolean limelightHasTarget;
    private double horizontalOffset;

    /**
     * Constructor.
     *
     * @param drivetrain The drivetrain subsystem
     * @param limelight The ball detection limelight
     */
    public DriveToHub(Drivetrain drivetrain,
        Lemonlight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.movePID = new PIDController(PIDValues.MOVE_P, PIDValues.MOVE_I, PIDValues.MOVE_D);
        this.alignPID = new PIDController(PIDValues.ALIGN_P, PIDValues.ALIGN_I, PIDValues.ALIGN_D);

        // TODO - set these
        movePID.setTolerance(1, 1);
        movePID.setSetpoint(DISTANCE_FROM_HUB);
        alignPID.setTolerance(1, 1);
        alignPID.setSetpoint(0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        movePID.reset();
        alignPID.reset();
    }

    @Override
    public void execute() {
        limelightHasTarget = limelight.hasTarget();
        limelightDistanceEstimate = Lemonlight.getLimelightDistanceEstimateIN(
            Lemonlight.MAIN_MOUNT_HEIGHT,
            Lemonlight.MAIN_MOUNT_ANGLE,
            Lemonlight.MAIN_TARGET_HEIGHT,
            limelight.getVerticalOffset());
        horizontalOffset = limelight.getHorizontalOffset();

        System.out.println("distance : " + limelightDistanceEstimate + "   hzo: " + horizontalOffset);

        if (limelightHasTarget) {
            double alignPower = alignPID.calculate(horizontalOffset);
            double movePower =  -Functions.clampDouble(movePID.calculate(limelightDistanceEstimate), 0.5, -0.5);

            System.out.println("align: " + alignPower + "   drive: " + movePower);
            drivetrain.setLeftMotorPower(movePower - alignPower);
            drivetrain.setRightMotorPower(movePower + alignPower);
            timer.reset();
            timer.stop();
        } else if (timer.get() > 1) {
            drivetrain.setLeftMotorPower(.1);
            drivetrain.setRightMotorPower(-.1);
        } else {
            timer.start();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return movePID.atSetpoint();
    }
}
