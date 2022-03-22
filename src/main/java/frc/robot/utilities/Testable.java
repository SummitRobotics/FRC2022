package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.LidarV4;
import java.util.HashMap;

/**
 * Interface for testing a subsystem.
 */
public interface Testable {

    public Subsystem getSubsystemObject();

    public CANSparkMax[] getMotors();

    public default ColorSensor[] getColorSensors() {
        return new ColorSensor[] {};
    }

    public default Lemonlight[] getLimelights() {
        return new Lemonlight[] {};
    }

    public default LidarV4[] getLidarV4s() {
        return new LidarV4[] {};
    }

    public default double getMotorTestSpeed() {
        return 0.3;
    }

    public default double getMotorTestRotations() {
        return 0.5;
    }

    public default double getAllowedTimeSeconds() {
        return 5;
    }

    public default HashMap<String, Boolean> runCustomTests() {
        return new HashMap<String, Boolean>();
    }
}
