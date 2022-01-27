package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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

    public void zeroEncoders() {
        leftMotorEncoder.setPosition(0);
        rightMotorEncoder.setPosition(0);
    }

    public double getLeftEncoderValue() {
        return leftMotorEncoder.getPosition();
    }

    public double getRightEncoderValue() {
        return rightMotorEncoder.getPosition();
    }

    public double getLeftMotorVelocity() {
        return leftMotorEncoder.getVelocity();
    }

    public double getRightMotorVelocity() {
        return rightMotorEncoder.getVelocity();
    }

    public boolean getPivotPos() {
        return pivotPos;
    }

    public boolean getLeftDetachPos() {
        return leftDetachPos;
    }

    public boolean getRightDetachPos() {
        return rightDetachPos;
    }

    public void setLeftMotorPower(double power) {
        leftMotor.set(leftMotorRateLimiter.getRateLimitedValue(power));
    }

    public void setRightMotorPower(double power) {
        rightMotor.set(rightMotorRateLimiter.getRateLimitedValue(power));
    }

    public void setMotorPower(double power) {
        setLeftMotorPower(power);
        setRightMotorPower(power);
    }

    public void setRightMotorPosition(double position) {
        rightPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setLeftMotorPosition(double position) {
        rightPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setMotorPosition(double position) {
        setRightMotorPosition(position);
        setLeftMotorPosition(position);
    }

    public void setPivotPos(boolean pos) {
        pivotPos = pos;
        pivot.set(pos);
    }

    public void togglePivotPos() {
        setPivotPos(!pivotPos);
    }

    public void setLeftDetachPos(boolean pos) {
        leftDetachPos = pos;
        leftDetach.set(pos);
    }

    public void toggleLeftDetachPos() {
        setLeftDetachPos(!leftDetachPos);
    }

    public void setRightDetachPos(boolean pos) {
        rightDetachPos = pos;
        rightDetach.set(pos);
    }

    public void toggleRightDetachPos() {
        setRightDetachPos(!rightDetachPos);
    }

    public void setDetachPos(boolean pos) {
        setRightDetachPos(pos);
        setLeftDetachPos(pos);
    }
}
