package frc.robot.subsystems;

import javax.lang.model.element.Element;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static final int lidarRegisterShooting = 36;
    private static final int lidarReliceShooting = 45;



    // tracker variables
    private String colorSensorMeasurement;
    private double lidarDistance;
    private double colorSensorDistance;
    private boolean isBallIndexed;
    private boolean doesBallExist;
    private boolean lidarAboveThreshold;

    private ConveyorState beltBall = ConveyorState.NONE;
    private ConveyorState indexBall = ConveyorState.NONE;


    // Constants storing acceptable distance data
    // TODO - set these
    public static final int
        MIN_EXISTS_LIDAR_DISTANCE = 1,
        MAX_EXISTS_LIDAR_DISTANCE = 62,
        //tune
        MAX_INDEXED_LIDAR_DISTANCE = 38,
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

        colorSensorMeasurement = "Unknown";
        lidarDistance = 1000;
        isBallIndexed = isBallIndexed();
        doesBallExist = doesBallExist();

        lidarAboveThreshold = false;
    
        beltBall = ConveyorState.NONE;
        indexBall = ConveyorState.NONE;

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
    public void periodic() {

        if (lidar == null || colorSensor == null) {
            return;
        }

        // Set tracker variables to prevent weird stuff from happening
        // if measurements change mid-cycle.
        colorSensorMeasurement = colorSensor.getColorString();
        lidarDistance = getLidarDistance();
        colorSensorDistance = colorSensor.getProximity();
        isBallIndexed = isBallIndexed();
        doesBallExist = doesBallExist();
        ConveyorState colorSensorBall = getColorSensorDetctedBall();

        //if no ball exists, clear everything
        if (!doesBallExist) {
            beltBall = ConveyorState.NONE;
            indexBall = ConveyorState.NONE;
            lidarAboveThreshold = false;

        } else {
            //if no ball in indexed, clear all index states
            if (wasBallShot()) {
                indexBall = ConveyorState.NONE;
            }

            //if color sensor reads a ball, make it the belts current ball
            //assumes belt is running foward
            //realy, we want another color sensor at index
            if (colorSensorBall != beltBall) {
                if (beltBall != ConveyorState.NONE) {
                    indexBall = beltBall;
                }
                beltBall = colorSensorBall;
            }
        }
    
        

    }

    /**
     * Returns the type of ball present in the position closer to the intake.
     *
     * @return the type of ball present in the position closer to the intake
     */
    public ConveyorState getBeltState() {
        return beltBall;
    }

    public ConveyorState getIndexState() {
        return indexBall;
    }

    private ConveyorState getColorSensorDetctedBall() {
        if (colorSensorMeasurement == "Unknown") {
            return ConveyorState.NONE;
        } else if (colorSensorMeasurement == "NoTarget") {
            return ConveyorState.NONE;
        } else if (colorSensorMeasurement == "Red") {
            return ConveyorState.RED;
        } else if (colorSensorMeasurement == "Blue") {
            return ConveyorState.BLUE;
        } else {
            return ConveyorState.NONE;
        }
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

    private boolean wasBallShot() {
        if (lidarDistance < lidarRegisterShooting) {
            lidarAboveThreshold = true;
        }

        if (lidarDistance > lidarReliceShooting) {
            if (lidarAboveThreshold) {
                lidarAboveThreshold = false;
                System.out.println("ball dec");
                return true;
            }
            lidarAboveThreshold = false;
        }

        return false;

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

    public int getLidarDistance() {
        return lidar.getAverageDistance();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("conveyor");
        // builder.addDoubleProperty("belt_motor_position", this::getBeltEncoderPosition, null);
        // builder.addDoubleProperty("index_motor_position", this::getIndexEncoderPosition, null);
        // builder.addDoubleProperty("belt_motor_speed", this::getBeltRPM, null);
        // builder.addDoubleProperty("index_motor_speed", this::getIndexRPM, null);
        builder.addDoubleProperty("lidar_distance", this::getLidarDistance, null);
        builder.addStringProperty("belt_ball", () -> this.getBeltState().toString(), null);
        builder.addStringProperty("index_ball", () -> this.getIndexState().toString(), null);
        //builder.addStringProperty("color_ball", () -> this.getColorSensorState().toString(), null);
        builder.addStringProperty("colorSensorRaw", colorSensor::getColorString, null);
        builder.addBooleanProperty("ball_exists", this::doesBallExist, null);
    }
}
