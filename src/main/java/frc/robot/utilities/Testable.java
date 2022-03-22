package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;

/**
 * Interface for testing a subsystem.
 */
public interface Testable {

    public Subsystem getSubsystemObject();

    public CANSparkMax[] getMotors();

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
