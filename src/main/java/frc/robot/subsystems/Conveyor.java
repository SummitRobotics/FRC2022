package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.ColorSensor;
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

    /**
    * Enum tracking what could be in the front or back of the conveyor.
    */
    public enum ConveyorState {
        NONE,
        BLUE,
        RED
    }

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
    private final ColorSensor colorSensor;
    private final LidarV3 lidar;

    // tracker variables
    private ConveyorState frontState;
    private ConveyorState backState;
    private String previousColorSensorMeasurement;
    private String colorSensorMeasurement;
    private double lidarDistance;
    private double colorSensorDistance;

    // Constants storing acceptable distance data
    private static final double
        MIN_EXISTS_LIDAR_DISTANCE = 0,
        MAX_EXISTS_LIDAR_DISTANCE = 0,
        MIN_QUEUED_LIDAR_DISTANCE = 0,
        MAX_QUEUED_LIDAR_DISTANCE = 0,
        MIN_COLOR_SENSOR_DISTANCE = 0,
        MAX_COLOR_SENSOR_DISTANCE = 0;

    /**
     * Subsystem to control the conveyor of the robot.
     *
     * @param colorSensor the color sensor
     * @param lidar the lidar
     */
    public Conveyor(ColorSensor colorSensor, LidarV3 lidar) {
        this.colorSensor = colorSensor;
        this.lidar = lidar;
        zeroEncoders();

        frontState = ConveyorState.NONE;
        backState = ConveyorState.NONE;
        previousColorSensorMeasurement = "Unknown";
        colorSensorMeasurement = "Unknown";
        lidarDistance = -1.0;
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
        previousColorSensorMeasurement = colorSensorMeasurement;
        colorSensorMeasurement = colorSensor.getColorString();
        lidarDistance = lidar.getAverageDistance();
        colorSensorDistance = colorSensor.getProximity();

        if (lidarDistance >= MAX_EXISTS_LIDAR_DISTANCE) {

            frontState = ConveyorState.NONE;
            backState = ConveyorState.NONE;

        } else if (colorSensorMeasurement != previousColorSensorMeasurement) {

            if (colorSensor.getColorString() == "Blue"
                && MIN_COLOR_SENSOR_DISTANCE <= colorSensorDistance
                && colorSensorDistance <= MAX_COLOR_SENSOR_DISTANCE) {

                backState = frontState;
                frontState = ConveyorState.BLUE;

            } else if (colorSensor.getColorString() == "Red"
                && MIN_COLOR_SENSOR_DISTANCE <= colorSensorDistance
                && colorSensorDistance <= MAX_COLOR_SENSOR_DISTANCE) {

                backState = frontState;
                frontState = ConveyorState.RED;

            } else if (!isBallQueued()) {

                backState = frontState;
                frontState = ConveyorState.NONE;
            }
        }
    }

    /**
     * Returns the type of ball present in the position closer to the intake.
     *
     * @return the type of ball present in the position closer to the intake
     */
    public ConveyorState getFront() {
        return frontState;
    }

    /**
     * Returns the type of ball present in the position further away from the intake.
     *
     * @return the type of ball present in the position further away from the intake
     */
    public ConveyorState getBack() {
        return backState;
    }

    /**
     * Returns whether or not there is a ball ready to be fired.
     * This should be checked by the shooter.
     *
     * @return whether or not there is a ball ready to be fired
     */
    public boolean isBallQueued() {
        if (MIN_QUEUED_LIDAR_DISTANCE <= lidarDistance
            && lidarDistance <= MAX_QUEUED_LIDAR_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns whether or not there is a single ball in the conveyor.
     * This should be checked by the shooter.
     *
     * @return whether or not there is a single ball in the conveyor
     */
    public boolean doesBallExist() {
        if (MIN_EXISTS_LIDAR_DISTANCE <= lidarDistance
            && lidarDistance <= MAX_EXISTS_LIDAR_DISTANCE) {
            return true;
        } else {
            return false;
        }
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
