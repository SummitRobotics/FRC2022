package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the intake of the robot.
 */
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

    // variable to track the intake solenoid's position.
    private boolean intakeSolenoidPosition = false;


    public Intake() {
        zeroEncoder();
    }

    /**
     * Sets the intake motor's power (between 1.0 and -1.0).
     *
     * @param power The intake motor's power, measured between 1.0 and -1.0
     */
    public void setIntakeMotorPower(double power) {
        intakeMotor.set(intakeRateLimiter.getRateLimitedValue(power));
    }

    /**
     * Gets the intake motor's encoder position (in rotations).
     *
     * @return position
     */
    public double getIntakeEncoderPosition() {
        return intakeEncoder.getPosition();
    }

    /**
     * Gets the intake motor's speed (in RPM).
     *
     * @return speed
     */
    public double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    /**
     * Manually overrides the intake motor's encoder position.
     *
     * @param position The desired position of the intake motor's encoder.
     */
    public void setIntakeEncoder(double position) {
        intakeEncoder.setPosition(position);
    }

    /**
     * Resets the intake motor's encoder position.
     */
    public void zeroEncoder() {
        setIntakeEncoder(0);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * Resets the motor's rate limiter (used for managing its acceleration and deceleration rates).
     */
    public void resetRateLimiter() {
        intakeRateLimiter.resetOld();
    }

    /**
     * Toggles the intake solenoid.
     */
    public void toggleIntakeSolenoid() {
        intakeSolenoidPosition = !intakeSolenoidPosition;
        intakeSolenoid.toggle();
    }

    /**
     * Sets the intake solenoid (true is open/extended).
     *
     * @param value The desired value of the intake solenoid (true is open/extended).
     */
    public void setIntakeSolenoid(boolean value) {
        intakeSolenoidPosition = value;
        intakeSolenoid.set(value);
    }

    /**
     * Gets the position of the intake solenoid (true is open/extended).
     *
     * @return position
     */
    public boolean getIntakeSolenoid() {
        return intakeSolenoidPosition;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("intake");
        builder.addDoubleProperty("intake_motor_position", this::getIntakeEncoderPosition, null);
        builder.addDoubleProperty("intake_motor_speed", this::getIntakeRPM, null);
        builder.addBooleanProperty("intake_solenoid_position", this::getIntakeSolenoid, null);
    }
}