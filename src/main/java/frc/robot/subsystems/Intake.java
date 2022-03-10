package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the intake of the robot.
 */
public class Intake extends SubsystemBase {

    /**
     * Enum for Intake States.
     */
    public enum States {
        UP,
        DOWN,
    }

    private States state;

    public static final double
            INTAKE_RATE = 0.5,
            INTAKE_MOTOR_SPEED = -0.5;

    // motor
    private final CANSparkMax intakeMotor =
        new CANSparkMax(Ports.INTAKE_MOTOR, MotorType.kBrushless);

    // solenoid
    private final Solenoid intakeSolenoid =
        new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.INTAKE_SOLENOID);

    // encoder
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    // variable to track the intake solenoid's position.
    private boolean intakeSolenoidPosition = false;


    /**
     * Subsystem to control the intake of the robot.
     */
    public Intake() {
        zeroEncoder();
        state = States.UP;
        intakeMotor.setOpenLoopRampRate(INTAKE_RATE);
    }

    /**
     * Sets the intake motor's power (between 1.0 and -1.0).
     *
     * @param power The intake motor's power, measured between 1.0 and -1.0
     */
    public void setIntakeMotorPower(double power) {
        intakeMotor.set(power);
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
     * Lowers intake.
     */

    public void lowerIntake() {
        if (state == States.UP) {
            setIntakeSolenoid(true);
            setIntakeMotorPower(INTAKE_MOTOR_SPEED);
        }
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
     * sets Intake State.
     *
     * @param states Sets the intake state
     */

    public void setState(States states) {
        state = states;
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
     * Toggles the intake solenoid.
     */
    public void toggleIntakeSolenoid() {
        setIntakeSolenoid(!intakeSolenoidPosition);
    }

    /**
     * Sets the intake solenoid (true is open/extended).
     *
     * @param value The desired value of the intake solenoid (true is open/extended).
     */
    public void setIntakeSolenoid(boolean value) {
        intakeSolenoidPosition = value;
        intakeSolenoid.set(value);
        updateState();
    }

    /**
     * Gets the position of the intake solenoid (true is open/extended).
     *
     * @return position
     */
    public boolean getIntakeSolenoid() {
        return intakeSolenoidPosition;
    }

    /**
     * Gets the current state of the intake.
     *
     * @return The current intake state.
     */
    public States getState() {
        return state;
    }

    private void updateState() {
        state = intakeSolenoidPosition ? States.DOWN : States.UP;
    }

    @Override
    public void periodic() {
        updateState();
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("intake");
        //builder.addDoubleProperty("intake_motor_position", this::getIntakeEncoderPosition, null);
        //builder.addDoubleProperty("intake_motor_speed", this::getIntakeRPM, null);
        builder.addBooleanProperty("intake_solenoid_position", this::getIntakeSolenoid, null);
    }
}
