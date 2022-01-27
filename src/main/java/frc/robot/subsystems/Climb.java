package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem for the Climb Subsystem.
 */
public class Climb extends SubsystemBase {

    public static final double
            P = 0,
            I = 0,
            D = 0,
            FF = 0,
            IZ = 0,
            LEFT_MOTOR_RATE = 0.01,
            RIGHT_MOTOR_RATE = 0.01;

    // Climb Motors
    private final CANSparkMax leftMotor =
            new CANSparkMax(Ports.LEFT_CLIMB_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor =
            new CANSparkMax(Ports.RIGHT_CLIMB_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    // Pivot Solenoid
    private final Solenoid pivot =
            new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.PIVOT_CLIMB_SOLENOID);

    // Detach Solenoids
    private final Solenoid leftDetach =
            new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.LEFT_DETACH_SOLENOID);
    private final Solenoid rightDetach =
            new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.RIGHT_DETACH_SOLENOID);

    // Solenoid Positions
    private boolean
            pivotPos = false,
            leftDetachPos = false,
            rightDetachPos = false;

    // PID Controllers
    private final SparkMaxPIDController leftPidController = leftMotor.getPIDController();
    private final SparkMaxPIDController rightPidController = rightMotor.getPIDController();

    // Encoders
    private final RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

    // Rate Limiters
    private final ChangeRateLimiter leftMotorRateLimiter = new ChangeRateLimiter(LEFT_MOTOR_RATE);
    private final ChangeRateLimiter rightMotorRateLimiter = new ChangeRateLimiter(RIGHT_MOTOR_RATE);

    /**
     * Public Constructor for climb subsystem.
     */
    public Climb() {
        leftPidController.setP(P);
        leftPidController.setI(I);
        leftPidController.setD(D);
        leftPidController.setFF(FF);
        leftPidController.setIZone(IZ);
        leftPidController.setOutputRange(-1.0, 1.0);

        rightPidController.setP(P);
        rightPidController.setI(I);
        rightPidController.setD(D);
        rightPidController.setFF(FF);
        rightPidController.setIZone(IZ);
        rightPidController.setOutputRange(-1.0, 1.0);

        zeroEncoders();
    }

    /**
     * Zeros the climb motor encoders.
     */
    public void zeroEncoders() {
        leftMotorEncoder.setPosition(0);
        rightMotorEncoder.setPosition(0);
    }

    /**
     * Gets the left climb encoder value.
     *
     * @return The number of rotations the left encoder has gone through
     *      since its last reset.
     */
    public double getLeftEncoderValue() {
        return leftMotorEncoder.getPosition();
    }

    /**
     * Gets the right climb encoder value.
     *
     * @return The number of rotations the right encoder has gone through
     *      since its last reset.
     */
    public double getRightEncoderValue() {
        return rightMotorEncoder.getPosition();
    }

    /**
     * Gets the RPM of the left climb motor.
     *
     * @return The current velocity of the left climb motor in RPM.
     */
    public double getLeftMotorVelocity() {
        return leftMotorEncoder.getVelocity();
    }

    /**
     * Gets the RPM of the right climb motor.
     *
     * @return The current velocity of the right climb motor in RPM.
     */
    public double getRightMotorVelocity() {
        return rightMotorEncoder.getVelocity();
    }

    /**
     * Gets the current position of the pneumatic pivot.
     *
     * @return The current position of the pivot piston.
     */
    public boolean getPivotPos() {
        return pivotPos;
    }

    /**
     * Gets the current position of the pneumatic detach for the left side.
     *
     * @return The current position of the left side detach piston.
     */
    public boolean getLeftDetachPos() {
        return leftDetachPos;
    }

    /**
     * Gets the current position of the pneumatic detach for the right side.
     *
     * @return Teh current position of the right side detach piston.
     */
    public boolean getRightDetachPos() {
        return rightDetachPos;
    }

    /**
     * Sets the left motor power.
     *
     * @param power The power to set the left motor.
     */
    public void setLeftMotorPower(double power) {
        leftMotor.set(leftMotorRateLimiter.getRateLimitedValue(power));
    }

    /**
     * Sets the right motor power.
     *
     * @param power The power to set the right motor.
     */
    public void setRightMotorPower(double power) {
        rightMotor.set(rightMotorRateLimiter.getRateLimitedValue(power));
    }

    /**
     * Sets the power for both of the climb motors.
     *
     * @param power The power to set both the climb motors.
     */
    public void setMotorPower(double power) {
        setLeftMotorPower(power);
        setRightMotorPower(power);
    }

    /**
     * Sets the position to move the right motor to.
     * This uses PID and requires accurate PID values.
     *
     * @param position The position to set the motor to. (Revolutions)
     */
    public void setRightMotorPosition(double position) {
        rightPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the position to move the left motor to.
     * This uses PID and requires accurate PID values.
     *
     * @param position The position to set the motor to. (Revolutions)
     */
    public void setLeftMotorPosition(double position) {
        rightPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the position to move both the motors to.
     * This uses PID and requires accurate PID values.
     *
     * @param position The position to set the motor to. (Revolutions)
     */
    public void setMotorPosition(double position) {
        setRightMotorPosition(position);
        setLeftMotorPosition(position);
    }

    /**
     * Sets the pivot position for the pneumatics.
     *
     * @param pos The position to set it to (Boolean)
     */
    public void setPivotPos(boolean pos) {
        pivotPos = pos;
        pivot.set(pos);
    }

    /**
     * Toggles the position of the pivot.
     */
    public void togglePivotPos() {
        setPivotPos(!pivotPos);
    }

    /**
     * Sets the position for the left detach pneumatics.
     *
     * @param pos The position to set the left detach pneumatics.
     */
    public void setLeftDetachPos(boolean pos) {
        leftDetachPos = pos;
        leftDetach.set(pos);
    }

    /**
     * Toggles the position of the left detach piston.
     */
    public void toggleLeftDetachPos() {
        setLeftDetachPos(!leftDetachPos);
    }

    /**
     * Sets the position for the right detach pneumatics.
     *
     * @param pos The position to set the right detach pneumatics.
     */
    public void setRightDetachPos(boolean pos) {
        rightDetachPos = pos;
        rightDetach.set(pos);
    }

    /**
     * Toggles the position of the right detach piston.
     */
    public void toggleRightDetachPos() {
        setRightDetachPos(!rightDetachPos);
    }

    /**
     * Sets the position for both the right and left detach pneumatics.
     *
     * @param pos The position to set the pistons to.
     */
    public void setDetachPos(boolean pos) {
        setRightDetachPos(pos);
        setLeftDetachPos(pos);
    }

    /**
     * Function to init telemetry for the climb subsystem.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climb");

        builder.addDoubleProperty("leftEncoderPosition", this::getLeftEncoderValue, null);
        builder.addDoubleProperty("rightEncoderPosition", this::getRightEncoderValue, null);
        builder.addDoubleProperty("leftMotorVelocity", this::getLeftMotorVelocity, null);
        builder.addDoubleProperty("rightMotorVelocity", this::getRightMotorVelocity, null);

        builder.addBooleanProperty("pivotPosition", this::getPivotPos, null);
        builder.addBooleanProperty("leftDetachPosition", this::getLeftDetachPos, null);
        builder.addBooleanProperty("rightDetachPosition", this::getRightDetachPos, null);
    }
}

