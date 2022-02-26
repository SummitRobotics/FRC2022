package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Drivetrain;

/**
 * Full auto intake mode.
 */
public class FullAutoIntake extends CommandBase {

    // PID values and constants
    // TODO - configure
    private static final double
        MOVE_P = 0,
        MOVE_I = 0,
        MOVE_D = 0,
        ALIGN_P = 0,
        ALIGN_I = 0,
        ALIGN_D = 0,
        DISTANCE_FROM_BALL = 0;

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
    public FullAutoIntake(Drivetrain drivetrain,
        Lemonlight limelight) {

        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);
        this.alignPID = new PIDController(ALIGN_P, ALIGN_I, ALIGN_D);

        // TODO - set these
        movePID.setTolerance(1, 1);
        movePID.setSetpoint(DISTANCE_FROM_BALL);
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
            Lemonlight.BALL_MOUNT_HEIGHT,
            Lemonlight.BALL_MOUNT_ANGLE,
            Lemonlight.BALL_TARGET_HEIGHT,
            limelight.getVerticalOffset());
        horizontalOffset = limelight.getHorizontalOffset();

        if (limelightHasTarget && !movePID.atSetpoint()) {
            double alignPower = alignPID.calculate(horizontalOffset);
            double movePower =  movePID.calculate(limelightDistanceEstimate);

            drivetrain.setLeftMotorPower(movePower + alignPower);
            drivetrain.setRightMotorPower(movePower - alignPower);
        }
    }

    @Override
    public boolean isFinished() {
        return !limelight.hasTarget()
            || movePID.atSetpoint();
    }
}
