package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.LidarV3;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the conveyor of the robot.
 */
public class Conveyor extends SubsystemBase {

    public static final double
            FRONT_RATE = 0.01,
            BACK_RATE = 0.01;

    // motors
    private final CANSparkMax front = new CANSparkMax(Ports.FRONT_CONVEYOR, MotorType.kBrushless);
    private final CANSparkMax back = new CANSparkMax(Ports.BACK_CONVEYOR, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder frontEncoder = front.getEncoder();
    private final RelativeEncoder backEncoder = back.getEncoder();

    // rate limiters
    private final ChangeRateLimiter frontRateLimiter = new ChangeRateLimiter(FRONT_RATE);
    private final ChangeRateLimiter backRateLimiter = new ChangeRateLimiter(BACK_RATE);

    // sensors
    private final LidarV3 lidar = new LidarV3();

    /**
     * An enum representing the different possible states of a given position in the conveyor.
     */
    public enum BallStates {
        NONE,
        BLUE,
        RED
    }

    public Conveyor() {
        zeroEncoders();
        lidar.startMeasuring();
    }

    /**
     * Sets the power of the front motor.
     *
     * @param power The power of the front motor (between 1 and -1).
     */
    public void setFrontMotorPower(double power) {
        front.set(frontRateLimiter.getRateLimitedValue(power));
    }

    /**
     * Sets the power of the back motor.
     *
     * @param power The power of the back motor (between 1 and -1).
     */
    public void setBackMotorPower(double power) {
        back.set(backRateLimiter.getRateLimitedValue(power));
    }

    /**
     * Sets the power of both motors.
     *
     * @param power the power to set the motor.
     */
    public void setMotorPower(double power) {
        setBackMotorPower(power);
        setFrontMotorPower(power);
    }

    /**
     * Gets the encoder position of the front motor (in rotations).
     *
     * @return position
     */
    public double getFrontEncoderPosition() {
        return frontEncoder.getPosition();
    }

    /**
     * Gets the encoder position of the back motor (in rotations).
     *
     * @return position
     */
    public double getBackEncoderPosition() {
        return backEncoder.getPosition();
    }

    /**
     * Manually sets the front encoder's position (in rotations).
     *
     * @param position The front encoder's position in rotations.
     */
    public void setFrontEncoder(double position) {
        frontEncoder.setPosition(position);
    }

    /**
     * Manually sets the back encoder's position (in rotations).
     *
     * @param position The back encoder's position in rotations.
     */
    public void setBackEncoder(double position) {
        backEncoder.setPosition(position);
    }

    /**
     * Gets the speed of the front motor (in RPM).
     *
     * @return speed The speed of the front motor in RPM.
     */
    public double getFrontRPM() {
        return frontEncoder.getVelocity();
    }

    /**
     * Gets the speed of the back motor (in RPM).
     *
     * @return speed The speed of the back motor in RPM.
     */
    public double getBackRPM() {
        return backEncoder.getVelocity();
    }

    /**
     * Resets the encoder values to 0.
     */
    public void zeroEncoders() {
        setFrontEncoder(0);
        setBackEncoder(0);
    }

    /**
     * Resets the rate limiter.
     * This should only be run when the motor is not moving.
     */
    public void resetRateLimiter() {
        frontRateLimiter.resetOld();
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        front.stopMotor();
        back.stopMotor();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("conveyor");
        builder.addDoubleProperty("front_motor_position", this::getFrontEncoderPosition, null);
        builder.addDoubleProperty("back_motor_position", this::getBackEncoderPosition, null);
        builder.addDoubleProperty("front_motor_speed", this::getFrontRPM, null);
        builder.addDoubleProperty("back_motor_speed", this::getBackRPM, null);
    }
}
