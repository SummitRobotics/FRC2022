package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.Lidar;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Interface for testing a subsystem.
 */
public interface Testable {

    public String getTestName();

    public default ArrayList<CANSparkMax> getMotors() {
        return new ArrayList<CANSparkMax>();
    }

    public default ArrayList<ColorSensor> getColorSensors() {
        return new ArrayList<ColorSensor>();
    }

    // The HashMap includes network table names and limelight objects
    public default HashMap<String, Lemonlight> getLimelights() {
        return new HashMap<String, Lemonlight>();
    }

    public default ArrayList<Lidar> getLidars() {
        return new ArrayList<Lidar>();
    }

    public default double getMotorTestSpeed() {
        return 0.3;
    }

    public default double getMotorTestRotations() {
        return 10;
    }

    public default double getAllowedTimeSeconds() {
        return 5;
    }

    public default double getMaxSensorLoopMilliseconds() {
        return 5;
    }

    public default HashMap<String, Boolean> initCustomTests() {
        return new HashMap<String, Boolean>();
    }

    public default HashMap<String, Boolean> runCustomTests() {
        return new HashMap<String, Boolean>();
    }
}
