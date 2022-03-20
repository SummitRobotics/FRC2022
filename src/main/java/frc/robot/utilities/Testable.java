package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for testing a subsystem.
 */
public interface Testable {

    public Subsystem getSubsystemObject();

    public CANSparkMax[] getMotors();
    
}
