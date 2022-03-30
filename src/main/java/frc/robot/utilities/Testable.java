package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.Lidar;
import java.util.HashMap;

/**
 * Interface for testing a subsystem.
 */
public interface Testable {

    public String getTestName();

    public default CANSparkMax[] getMotors() {
        return new CANSparkMax[] {};
    }

    public default ColorSensor[] getColorSensors() {
        return new ColorSensor[] {};
    }

    // The HashMap includes network table names and limelight objects
    public default HashMap<String, Lemonlight> getLimelights() {
        return new HashMap<String, Lemonlight>();
    }

    public default Lidar[] getLidars() {
        return new Lidar[] {};
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
