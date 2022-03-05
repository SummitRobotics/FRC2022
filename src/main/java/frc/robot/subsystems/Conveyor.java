package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the conveyor of the robot.
 */
public class Conveyor extends SubsystemBase {

    // TODO - Set these
    public static final double
            BELT_RATE = 0.01,
            INDEX_RATE = 0.01,
            P = 0,
            I = 0,
            D = 0,
            FF = 0,
            IZ = 0;

    /**
    * Enum tracking what could be in the front or back of the conveyor.
    */
    public enum ConveyorState {
        NONE,
        BLUE,
        RED
    }


    // motors
    private final CANSparkMax belt = new CANSparkMax(Ports.FRONT_CONVEYOR, MotorType.kBrushless);
    private final CANSparkMax index = new CANSparkMax(Ports.BACK_CONVEYOR, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder beltEncoder = belt.getEncoder();
    private final RelativeEncoder indexEncoder = index.getEncoder();

    // PID controllers
    private final SparkMaxPIDController indexPID = index.getPIDController();

    // tracker variables
    private String previousColorSensorMeasurement;
    private String colorSensorMeasurement;
    private double lidarDistance;
    private double previousLidarDistance;
    private double colorSensorDistance;
    private boolean isBallIndexed;
    private boolean wasBallIndexed;
    private boolean doesBallExist;
    private double beltRPM;
    private double indexRPM;

    // Constants storing acceptable distance data
    // TODO - set these
    private static final double
        MIN_EXISTS_LIDAR_DISTANCE = 1,
        MAX_EXISTS_LIDAR_DISTANCE = 65,
        MAX_INDEXED_LIDAR_DISTANCE = 35,
        MIN_INDEXED_LIDAR_DISTANCE = 1;

    /**
     * Subsystem to control the conveyor of the robot.
     */
    public Conveyor() {
        zeroEncoders();

        indexPID.setP(P);
        indexPID.setI(I);
        indexPID.setD(D);
        indexPID.setFF(FF);
        indexPID.setIZone(IZ);
        indexPID.setOutputRange(-1.0, 1.0);

        previousColorSensorMeasurement = "Unknown";
        colorSensorMeasurement = "Unknown";
        previousLidarDistance = -1.0;
        lidarDistance = -1.0;
        wasBallIndexed = false;
        beltRPM = 0;
        indexRPM = 0;
        index.setInverted(true);
    }

    /**
     * Sets the power of the belt motor.
     *
     * @param power The rate-limited power of the belt motor (between 1 and -1).
     */
    public void setBeltMotorPower(double power) {
        belt.set(power);
    }

    /**
     * Sets the power of the index motor.
     *
     * @param power The rate-limited power of the index motor (between 1 and -1).
     */
    public void setIndexMotorPower(double power) {
        index.set(power);
    }

    /**
     * Gets the encoder position of the belt motor (in rotations).
     *
     * @return position
     */
    public double getBeltEncoderPosition() {
        return beltEncoder.getPosition();
    }

    /**
     * Gets the encoder position of the index motor (in rotations).
     *
     * @return position
     */
    public double getIndexEncoderPosition() {
        return indexEncoder.getPosition();
    }

    /**
     * Manually sets the belt encoder's position (in rotations).
     *
     * @param position The belt encoder's position in rotations.
     */
    public void setBeltEncoder(double position) {
        beltEncoder.setPosition(position);
    }

    /**
     * Manually sets the index encoder's position (in rotations).
     *
     * @param position The index encoder's position in rotations.
     */
    public void setIndexEncoder(double position) {
        indexEncoder.setPosition(position);
    }

    /**
     * Sets the target position of the index motor.
     *
     * @param position The desired position of the index motor.
     */
    public void setIndexTargetPosition(double position) {
        indexPID.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Gets the speed of the belt motor (in RPM).
     *
     * @return speed The speed of the front motor in RPM.
     */
    public double getBeltRPM() {
        return beltEncoder.getVelocity();
    }

    /**
     * Gets the speed of the index motor (in RPM).
     *
     * @return speed The speed of the back motor in RPM.
     */
    public double getIndexRPM() {
        return indexEncoder.getVelocity();
    }

    /**
     * Resets the encoder values to 0.
     */
    public void zeroEncoders() {
        setBeltEncoder(0);
        setIndexEncoder(0);
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        belt.stopMotor();
        index.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("conveyor");
        //builder.addDoubleProperty("belt_motor_position", this::getBeltEncoderPosition, null);
        //builder.addDoubleProperty("index_motor_position", this::getIndexEncoderPosition, null);
        //builder.addDoubleProperty("belt_motor_speed", this::getBeltRPM, null);
        //builder.addDoubleProperty("index_motor_speed", this::getIndexRPM, null);
        // builder.addStringProperty("belt_ball", () -> this.getBeltState().toString(), null);
        // builder.addStringProperty("index_ball", () -> this.getIndexState().toString(), null);
        // builder.addStringProperty("color_ball", () -> this.getColorSensorState().toString(), null);
        // builder.addBooleanProperty("ball_exists", this::doesBallExist, null);
    }
}
