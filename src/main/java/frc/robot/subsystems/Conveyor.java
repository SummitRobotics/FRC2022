package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.LidarV3;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the conveyor of the robot.
 */
public class Conveyor extends SubsystemBase {

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

    // sensors
    private final ColorSensor colorSensor;
    private final LidarV3 lidar;

    // tracker variables
    private ConveyorState beltState;
    private ConveyorState indexState;
    private String previousColorSensorMeasurement;
    private String colorSensorMeasurement;
    private double lidarDistance;
    private double colorSensorDistance;
    private boolean isBallIndexed;
    private boolean wasBallIndexed;
    private boolean doesBallExist;
    private double beltRPM;
    private double indexRPM;

    // Constants storing acceptable distance data
    private static final double
        MIN_EXISTS_LIDAR_DISTANCE = 0,
        MAX_EXISTS_LIDAR_DISTANCE = 0,
        MAX_INDEXED_LIDAR_DISTANCE = 0,
        MIN_INDEXED_LIDAR_DISTANCE = 0,
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

        beltState = ConveyorState.NONE;
        indexState = ConveyorState.NONE;
        previousColorSensorMeasurement = "Unknown";
        colorSensorMeasurement = "Unknown";
        lidarDistance = -1.0;
        wasBallIndexed = false;
        isBallIndexed = getIsBallIndexed();
        doesBallExist = getDoesBallExist();
        beltRPM = 0;
        indexRPM = 0;
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
    public void periodic() {

        // Set tracker variables to prevent weird stuff from happening
        // if measurements change mid-cycle.
        previousColorSensorMeasurement = colorSensorMeasurement;
        colorSensorMeasurement = colorSensor.getColorString();
        lidarDistance = lidar.getAverageDistance();
        colorSensorDistance = colorSensor.getProximity();
        wasBallIndexed = isBallIndexed;
        isBallIndexed = getIsBallIndexed();
        doesBallExist = getDoesBallExist();
        beltRPM = getBeltRPM();
        indexRPM = getIndexRPM();

        if (!doesBallExist) {

            // If no balls are detected, both states are set to none.
            beltState = ConveyorState.NONE;
            indexState = ConveyorState.NONE;

        } else if (beltRPM > 0) {
            // If the belt is moving forwards...

            if (colorSensorMeasurement != previousColorSensorMeasurement && MIN_COLOR_SENSOR_DISTANCE <= colorSensorDistance && colorSensorDistance <= MAX_COLOR_SENSOR_DISTANCE) {
                // If we detect a different measurement and it seems valid...

                if (colorSensor.getColorString() == "Blue") {
                    // If the measurement is blue...
                    
                    // Update the states based on what they currently are.
                    if (indexState == ConveyorState.NONE) {
                        indexState = beltState;
                        beltState = ConveyorState.BLUE;
                    } else {
                        beltState = ConveyorState.BLUE;
                    }

                } else if (colorSensor.getColorString() == "Red") {
                    // If the measurement is red...

                    // Update the states based on what they currently are.
                    if (indexState == ConveyorState.NONE) {
                        indexState = beltState;
                        beltState = ConveyorState.RED;
                    } else {
                        beltState = ConveyorState.RED;
                    }
                }
            }
            
            if (indexRPM > 0 && indexState != ConveyorState.NONE && wasBallIndexed && !isBallIndexed) {
                // If we probably fired the indexed ball...

                // Update the ball states.
                indexState = beltState;
                beltState = ConveyorState.NONE;
            }

        } else if (beltRPM < 0) {
            // If the belt was manually overridden to run backwards...

            if (colorSensorMeasurement != previousColorSensorMeasurement && MIN_COLOR_SENSOR_DISTANCE <= colorSensorDistance && colorSensorDistance <= MAX_COLOR_SENSOR_DISTANCE) {
                // If we detect a different measurement and it seems valid...

                if ( (colorSensorMeasurement == "Blue" && beltState == ConveyorState.BLUE) || (colorSensorMeasurement == "Red"&& beltState == ConveyorState.RED) ) {
                    // If that measurement matches beltState...

                    beltState = ConveyorState.NONE;

                } else if (beltState == ConveyorState.NONE && ( (colorSensorMeasurement == "Red" && indexState == ConveyorState.RED && !isBallIndexed) || (colorSensorMeasurement == "Blue" && indexState == ConveyorState.BLUE && !isBallIndexed) ) ) {
                    // If that measurement matches indexState, the ball is not yet indexed, and
                    // beltState is currently empty...

                    indexState = ConveyorState.NONE;

                }
            }
            
            if (indexRPM > 0 && indexState != ConveyorState.NONE && wasBallIndexed && !isBallIndexed) {
                // If we probably fired the indexed ball...

                indexState = ConveyorState.NONE;

            }

        } else {

            if (indexRPM > 0 && indexState != ConveyorState.NONE && wasBallIndexed && !isBallIndexed) {
                // If the belt is stationary, but we probably still fired the indexed ball...

                indexState = ConveyorState.NONE;

            }
        }
    }

    /**
     * Returns the type of ball present in the position closer to the intake.
     *
     * @return the type of ball present in the position closer to the intake
     */
    public ConveyorState getBeltState() {
        return beltState;
    }

    /**
     * Returns the type of ball present in the position further away from the intake.
     * Note: To keep the logic simple, the conveyor code automatically assigns the
     * furthest-along ball to the indexed state, even if it has not gotten off the conveyor yet.
     * Check isBallIndexed to see if the ball is actually ready.
     *
     * @return the type of ball present in the position further away from the intake
     */
    public ConveyorState getWillBeIndexedState() {
        return indexState;
    }

    /**
     * Returns whether or not there is a ball ready to be fired.
     * This should be checked by the shooter.
     *
     * @return whether or not there is a ball ready to be fired
     */
    public boolean getIsBallIndexed() {
        if (MIN_INDEXED_LIDAR_DISTANCE <= lidarDistance && lidarDistance <= MAX_INDEXED_LIDAR_DISTANCE) {
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
    public boolean getDoesBallExist() {
        if (MIN_EXISTS_LIDAR_DISTANCE <= lidarDistance && lidarDistance <= MAX_EXISTS_LIDAR_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }

    public double getLidarDistance() {
        return lidar.getAverageDistance();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("conveyor");
        builder.addDoubleProperty("belt_motor_position", this::getBeltEncoderPosition, null);
        builder.addDoubleProperty("index_motor_position", this::getIndexEncoderPosition, null);
        builder.addDoubleProperty("belt_motor_speed", this::getBeltRPM, null);
        builder.addDoubleProperty("index_motor_speed", this::getIndexRPM, null);
        builder.addStringProperty("belt_ball", () -> this.getBeltState().toString(), null);
        builder.addStringProperty("will_be_indexed_ball",
            () -> this.getWillBeIndexedState().toString(), null);
    }
}
