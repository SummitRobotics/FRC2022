package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.Lidar;
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

    // sensors
    private final ColorSensor colorSensor;
    private final Lidar lidar;

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
        MAX_INDEXED_LIDAR_DISTANCE = 32,
        MIN_INDEXED_LIDAR_DISTANCE = 1;

    /**
     * Subsystem to control the conveyor of the robot.
     *
     * @param colorSensor the color sensor
     * @param lidar the lidar
     */
    public Conveyor(ColorSensor colorSensor, Lidar lidar) {
        this.colorSensor = colorSensor;
        this.lidar = lidar;
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
        isBallIndexed = isBallIndexed();
        doesBallExist = doesBallExist();
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

    private Ball colorSensorState = null;
    private Ball lidarBind = null;
    private Ball beltState = null;
    private Ball indexState = null;

    @Override
    public void periodic() {

        if (lidar == null || colorSensor == null) {
            return;
        }

        // Set tracker variables to prevent weird stuff from happening
        // if measurements change mid-cycle.
        previousColorSensorMeasurement = colorSensorMeasurement;
        colorSensorMeasurement = colorSensor.getColorString();
        previousLidarDistance = lidarDistance;
        lidarDistance = lidar.getAverageDistance();
        colorSensorDistance = colorSensor.getProximity();
        wasBallIndexed = isBallIndexed;
        isBallIndexed = isBallIndexed();
        doesBallExist = doesBallExist();
        beltRPM = -getBeltRPM();
        indexRPM = -getIndexRPM();

        if (!doesBallExist) {

            // If no balls are detected, both states are set to none.
            colorSensorState = null;
            beltState = null;
            indexState = null;
            lidarBind = null;

        } else if (beltRPM > 0) {
            // If the belt is moving forwards...

            if (colorSensorMeasurement != previousColorSensorMeasurement || (colorSensorMeasurement != "NoTarget" && colorSensorState == null)) {
                // If we detect a different measurement

                if (colorSensor.getColorString() == "Blue") {
                    // If the measurement is blue...
                    colorSensorState = new Ball(true);

                    // If the lidar is not bound it to the ball.
                    if (lidarBind == null) {
                        lidarBind = colorSensorState;
                    }

                } else if (colorSensor.getColorString() == "Red") {
                    // If the measurement is red...
                    colorSensorState = new Ball(false);

                    // If the lidar is not bound it to the ball.
                    if (lidarBind == null) {
                        lidarBind = colorSensorState;
                    }
                } else {
                    // If the measurement is not Red or Blue move to the next stage.
                    if (beltState == null) {
                        beltState = colorSensorState;
                        colorSensorState = null;
                    }
                }
            }
            if (lidarBind != null
                    && indexState == null
                    && lidarDistance <= MAX_INDEXED_LIDAR_DISTANCE
                    && lidarDistance >= MIN_INDEXED_LIDAR_DISTANCE) {
                if (beltState == lidarBind) {
                    // If the ball is in the belt.
                    indexState = beltState;
                    beltState = null;
                } else if (colorSensorState == lidarBind) {
                    // If the ball is in the color sensor state.
                    indexState = colorSensorState;
                    colorSensorState = null;
                }
            }
            if (indexRPM > 0
                    && indexState != null
                    && wasBallIndexed
                    && !isBallIndexed
                    && lidarDistance - previousLidarDistance > 5) {
                indexState = null;
                if (beltState != null) {
                    lidarBind = beltState;
                } else if (colorSensorState != null) {
                    lidarBind = colorSensorState;
                } else {
                    lidarBind = null;
                }
            }
        } else if (beltRPM < 0) {
            System.out.println("TEST");
        }
    }

    /**
     * Returns the type of ball present in the position closer to the intake.
     *
     * @return the type of ball present in the position closer to the intake
     */
    public ConveyorState getBeltState() {
        return beltState == null ? ConveyorState.NONE : beltState.toConveyorState();
    }

    public ConveyorState getIndexState() {
        return indexState == null ? ConveyorState.NONE : indexState.toConveyorState();
    }

    public ConveyorState getColorSensorState() {
        return colorSensorState == null ? ConveyorState.NONE : colorSensorState.toConveyorState();
    }

    /**
     * Returns whether or not there is a ball ready to be fired.
     * This should be checked by the shooter.
     *
     * @return whether or not there is a ball ready to be fired
     */
    public boolean isBallIndexed() {
        if (lidar == null || colorSensor == null) {
            return false;
        }
        if (MIN_INDEXED_LIDAR_DISTANCE <= lidarDistance
            && lidarDistance <= MAX_INDEXED_LIDAR_DISTANCE) {
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
        if (lidar == null || colorSensor == null) {
            return false;
        }
        if (MIN_EXISTS_LIDAR_DISTANCE <= lidarDistance
            && lidarDistance <= MAX_EXISTS_LIDAR_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }

    public double getLidarDistance() {
        return lidar.getAverageDistance();
    }

    private class Ball {
        boolean isBlue;
        //boolean isRed;

        Ball(boolean isBlue) {
            this.isBlue = isBlue;
            //this.isRed = !isBlue;
        }

        public ConveyorState toConveyorState() {
            return isBlue ? ConveyorState.BLUE : ConveyorState.RED;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("conveyor");
        //builder.addDoubleProperty("belt_motor_position", this::getBeltEncoderPosition, null);
        //builder.addDoubleProperty("index_motor_position", this::getIndexEncoderPosition, null);
        //builder.addDoubleProperty("belt_motor_speed", this::getBeltRPM, null);
        //builder.addDoubleProperty("index_motor_speed", this::getIndexRPM, null);
        builder.addStringProperty("belt_ball", () -> this.getBeltState().toString(), null);
        builder.addStringProperty("index_ball", () -> this.getIndexState().toString(), null);
        builder.addStringProperty("color_ball", () -> this.getColorSensorState().toString(), null);
        builder.addBooleanProperty("ball_exists", this::doesBallExist, null);
    }
}
