package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.lists.Ports;

public class Intake extends SubsystemBase {

    public static final double INTAKE_RATE = 0.01;

    // motor
    private final CANSparkMax intakeMotor =
        new CANSparkMax(Ports.INTAKE_MOTOR, MotorType.kBrushless);

    // solenoid
    private final Solenoid intakeSolenoid =
        new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.INTAKE_SOLENOID);

    // encoder
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    // rate limiter
    private final ChangeRateLimiter intakeRateLimiter = new ChangeRateLimiter(INTAKE_RATE);

    private boolean intakeSolenoidPosition = false;


    public Intake() {
        zeroEncoder();
    }

    public void setIntakeMotorPower(double power) {
        intakeMotor.set(intakeRateLimiter.getRateLimitedValue(power));
    }

    public double getIntakeEncoderPosition() {
        return intakeEncoder.getPosition();
    }

    public double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    public void setIntakeEncoder(double position) {
        intakeEncoder.setPosition(position);
    }

    public void zeroEncoder() {
        setIntakeEncoder(0);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public void resetRateLimiter() {
        intakeRateLimiter.resetOld();
    }

    public void toggleIntakeSolenoid() {
        intakeSolenoidPosition = !intakeSolenoidPosition;
        intakeSolenoid.toggle();
    }

    public void setIntakeSolenoid(boolean value) {
        intakeSolenoidPosition = value;
        intakeSolenoid.set(value);
    }

    public boolean getIntakeSolenoid() {
        return intakeSolenoidPosition;
    }
}
