package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.Lemonlight;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Testable;
import frc.robot.utilities.lists.Ports;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Subsystem to control the intake of the robot.
 */
public class Intake extends SubsystemBase implements Testable {
    public static final double 
        P = 0.004,
        I = 0.0,
        D = 0.0;

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
            INTAKE_MOTOR_SPEED = .5, 
            MAX_RPM = 8000, 
            ACTUAL_SPEED = MAX_RPM / 2,
            MOVE_TO_CONVEYOR = ACTUAL_SPEED / 2;

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

    // ball limelight (tested in this subsystem)
    private Lemonlight ballLimelight;

    private final SparkMaxPIDController intakePidController = intakeMotor.getPIDController();

    /**
     * Subsystem to control the intake of the robot.
     *
     * @param ballLimelight The limelight used for ball tracking
     */
    public Intake(Lemonlight ballLimelight) {
        this.ballLimelight = ballLimelight;
        intakePidController.setP(P);
        intakePidController.setI(I);
        intakePidController.setD(D);
        intakePidController.setOutputRange(-1.0, 1.0);
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
     * Makes the intake motor rotate to a postion.
     *
     * @param position the postion to be rotated to
     */

    public void setIntakePosition(double position) {
        intakePidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
    
    /**
     * Set the speed of the intake.
     *
     * @param speed The desired intake speed
     */
    public void setIntakeSpeed(double speed) {
        speed = Functions.clampDouble(speed, MAX_RPM, -MAX_RPM);
        intakePidController.setReference(speed, CANSparkMax.ControlType.kVelocity);

    }

    /**
     * Resets the intake motor's encoder position.
     */

    public void zeroEncoder() {
        setIntakeEncoder(0);
    }

    /**
     * gets intake motor power.
     *
     * @return power draw
     */
    public double getIntakePower() {
        return intakeMotor.getOutputCurrent();
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
    public String getTestName() {
        return "Intake";
    }

    @Override
    public ArrayList<CANSparkMax> getMotors() {
        ArrayList<CANSparkMax> result = new ArrayList<CANSparkMax>();
        result.add(intakeMotor);
        return result;
    }

    @Override
    public HashMap<String, Lemonlight> getLimelights() {
        HashMap<String, Lemonlight> result = new HashMap<String, Lemonlight>();
        result.put("limelight-balls", ballLimelight);
        return result;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("intake");
        builder.addDoubleProperty("intake_motor_position", this::getIntakeEncoderPosition, null);
        builder.addDoubleProperty("intake_motor_speed", this::getIntakeRPM, null);
        builder.addDoubleProperty("motor_current", this::getIntakePower, null);
        builder.addBooleanProperty("intake_solenoid_position", this::getIntakeSolenoid, null);
    }
}
