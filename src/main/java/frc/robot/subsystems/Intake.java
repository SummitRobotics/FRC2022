package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Ports;

public class Intake extends SubsystemBase {

    // motor
    private final CANSparkMax intakeMotor =
        new CANSparkMax(Ports.INTAKE_MOTOR, MotorType.kBrushless);

    // solenoid
    private final Solenoid intakeSolenoid =
        new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.INTAKE_SOLENOID);

    // encoder
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    public Intake() {}
    
}
