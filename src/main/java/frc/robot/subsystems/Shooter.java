package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem for the Shooter mechanism.
 */
public class Shooter extends SubsystemBase {

    public static final double
            P = 0,
            I = 0,
            D = 0,
            FF = 0,
            IZ = 0,
            MAX_RPM = 0;

    private final CANSparkMax shooterMotor = new CANSparkMax(
            Ports.SHOOTER_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);

    private final DoubleSolenoid hood = new DoubleSolenoid(
            Ports.PCM_1,
            PneumaticsModuleType.REVPH,
            Ports.HOOD_SOLENOID_DOWN,
            Ports.HOOD_SOLENOID_UP);

    // Pid controller
    private final SparkMaxPIDController shooterMotorPIDController = shooterMotor.getPIDController();

    // Encoder
    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

    // Hood position false - Piston not extended : true - Piston extended
    private boolean hoodPos = false;

    /**
     * Creates a new shooter instance.
     */
    public Shooter() {
        shooterMotorPIDController.setP(P);
        shooterMotorPIDController.setI(I);
        shooterMotorPIDController.setD(D);
        shooterMotorPIDController.setFF(FF);
        shooterMotorPIDController.setIZone(IZ);
        shooterMotorPIDController.setOutputRange(-1.0, 1.0);
    }

    /**
     * Sets the motor speed from -1 > value > 1.
     *
     * @param power speed to set the motor to
     */
    public void setMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        shooterMotor.set(power);
    }

    /**
     * Sets the motor to run at a specific voltage.
     *
     * @param volts The voltage.
     */
    public void setMotorVolts(double volts) {
        shooterMotor.setVoltage(volts);
    }

    /**
     * Sets the motor to run to a target speed in RPM.
     *
     * @param speed The target speed in RPM.
     */
    public void setMotorTargetSpeed(double speed) {
        speed = Functions.clampDouble(speed, MAX_RPM, -MAX_RPM);
        shooterMotorPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Sets the shooter encoder value.
     *
     * @param position Value to set it to
     */
    public void setEncoderValue(double position) {
        shooterEncoder.setPosition(position);
    }

    /**
     * Gets the shooter encoder value.
     *
     * @return the encoder value
     */
    public double getEncoderValue() {
        return shooterEncoder.getPosition();
    }

    /**
     * Zeros the shooter encoder.
     */
    public void zeroEncoder() {
        setEncoderValue(0);
    }

    /**
     * Gets the current shooter position in rotations.
     *
     * @return The current shooter position
     */
    public double getShooterPosition() {
        return shooterEncoder.getPosition();
    }

    /**
     * Gets the current shooter velocity in RPM.
     *
     * @return the shooter velocity in RPM
     */
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Sets the ramp rate in a closed environment.
     * EG. When under PID
     *
     * @param rate The rate to set it to.
     */
    public void setClosedRampRate(double rate) {
        shooterMotor.setClosedLoopRampRate(rate);
    }

    /**
     * Sets the ramp rate in an open environment.
     * EG. When setting the motor power
     *
     * @param rate The rate to set it to.
     */
    public void setOpenRampRate(double rate) {
        shooterMotor.setOpenLoopRampRate(rate);
    }

    /**
     * Gets the current PID values.
     *
     * @return the pid values as an array of doubles.
     */
    public double[] getPID() {
        return new double[] {P, I, D};
    }

    /**
     * Extends the shooter hood.
     */
    public void extendHood() {
        hoodPos = true;
        hood.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Retracts the shooter hood.
     */
    public void retractHood() {
        hoodPos = false;
        hood.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Gets the current hood position.
     * True - Piston Extended
     * False - Piston Not Extended
     *
     * @return The current hood position.
     */
    public boolean getHoodPos() {
        return hoodPos;
    }

    /**
     * Toggles the hoods position.
     * If the hood was retracted it will extend it and vice versa.
     */
    public void toggleHoodPos() {
        if (hoodPos) {
            hoodPos = false;
            hood.set(DoubleSolenoid.Value.kReverse);
        } else {
            hoodPos = true;
            hood.set(DoubleSolenoid.Value.kForward);
        }
    }
}