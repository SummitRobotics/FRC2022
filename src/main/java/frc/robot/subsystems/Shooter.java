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
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem for the Shooter mechanism.
 */
public class Shooter extends SubsystemBase {

    // TODO - Set these
    public static final double
            P = 0,
            I = 0,
            D = 0,
            FF = 0,
            IZ = 0,
            MAX_RPM = 0,
            RATE = 0.05;


    private final CANSparkMax shooterMotor = new CANSparkMax(
            Ports.SHOOTER_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);

    private final Solenoid hood = new Solenoid(
            Ports.PCM_1,
            PneumaticsModuleType.REVPH,
            Ports.HOOD_SOLENOID);

    // Pid controller
    private final SparkMaxPIDController shooterMotorPIDController = shooterMotor.getPIDController();

    // Encoder
    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

    // Hood position false - Piston not extended : true - Piston extended
    private boolean hoodPos = false;

    // Rate Limiter
    private ChangeRateLimiter motorRateLimiter = new ChangeRateLimiter(RATE);

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
        
        zeroEncoders();
    }

    /**
     * Sets the motor speed from -1 < value < 1.
     *
     * @param power speed to set the motor to
     */
    public void setMotorPower(double power) {
        shooterMotor.set(motorRateLimiter.getRateLimitedValue(power));
    }
    
    /**
     * Stops the motor.
     */
    public void stop() {
        setMotorPower(0);
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
     * Zeros the shooter encoder(s).
     * Used the plural version of encoder for consistancy throughout the program.
     */
    public void zeroEncoders() {
        setEncoderValue(0);
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
     * Gets the current PID values.
     *
     * @return the pid values as an array of doubles.
     */
    public double[] getPID() {
        return new double[] {P, I, D, FF, IZ};
    }

    /**
     * Extends the shooter hood.
     */
    public void extendHood() {
        hoodPos = true;
        hood.set(true);
    }

    /**
     * Retracts the shooter hood.
     */
    public void retractHood() {
        hoodPos = false;
        hood.set(false);
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
     * Sets the hoods position.
     * True - Hood is extended
     * False - Hood is retracted
     *
     * @param hoodPos The position to set the hood to
     */
    public void setHoodPos(boolean hoodPos) {
        this.hoodPos = hoodPos;
        hood.set(hoodPos);
    }

    /**
     * Toggles the hoods position.
     * If the hood was retracted it will extend it and vice versa.
     */
    public void toggleHoodPos() {
        setHoodPos(!hoodPos);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");

        builder.addDoubleProperty("encoderValue", this::getEncoderValue, null);
        builder.addDoubleProperty("shooterVelocity", this::getShooterVelocity, null);
        builder.addBooleanProperty("hoodPosition", this::getHoodPos, null);
    }
}