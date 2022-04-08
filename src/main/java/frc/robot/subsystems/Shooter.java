package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.Lemonlight;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Testable;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.Ports;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Subsystem for the Shooter mechanism.
 */
public class Shooter extends SubsystemBase implements Testable {

    /**
     * Enum describing shooter state.
     */
    public enum States {
        NOT_SHOOTING,
        NO_BALL,
        NO_TARGET,
        DRIVING_AND_ALIGNING,
        SETTING_HOOD,
        SPOOLING,
        READY_TO_FIRE
    }

    private States shooterState = States.NOT_SHOOTING;

    // TODO - Set these
    public static final double
            P = 0.0005,
            I = 0,
            D = 0,
            FF = 0.000204,
            IZ = 0,
            MAX_RPM = 5000;

    PIDController pidDum = new PIDController(1.4217E-11, 0, 0);
    SimpleMotorFeedforward ffDum = new SimpleMotorFeedforward(0.5235, 0.0666, 0.011555);

    private final CANSparkMax shooterMotorMain = new CANSparkMax(
            Ports.SHOOTER_MOTOR_1,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooterMotorFollow = new CANSparkMax(
        Ports.SHOOTER_MOTOR_2,
        CANSparkMaxLowLevel.MotorType.kBrushless
    );

    private final Solenoid hood = new Solenoid(
            Ports.PCM_1,
            PneumaticsModuleType.REVPH,
            Ports.HOOD_SOLENOID);

    // Pid controller
    private final SparkMaxPIDController shooterMotorPIDController = shooterMotorMain.getPIDController();

    // Encoder
    private final RelativeEncoder shooterEncoder = shooterMotorMain.getEncoder();

    // Hood position false - Piston not extended : true - Piston extended
    private boolean hoodPos = false;

    private LEDCall firing = new LEDCall(LEDPriorities.SHOOTING, LEDRange.All).solid(Colors.PURPLE);

    // targeting limelight (for testing)
    private Lemonlight targetingLimelight;
    
    private double setPoint = 0;

    /**
     * Creates a new shooter instance.
     *
     * @param targetingLimelight The limelight used for targeting
     */
    public Shooter(Lemonlight targetingLimelight) {
        this.targetingLimelight = targetingLimelight;
        shooterMotorPIDController.setP(P);
        shooterMotorPIDController.setI(I);
        shooterMotorPIDController.setD(D);
        shooterMotorPIDController.setFF(FF);
        shooterMotorPIDController.setIZone(IZ);
        shooterMotorPIDController.setOutputRange(-1.0, 1.0);
        shooterMotorMain.enableVoltageCompensation(12);
        
        zeroEncoders();
        shooterMotorFollow.follow(shooterMotorMain, true);
        setPoint = 0;
    }

    /**
     * sets the state of the shooter.
     *
     * @param state the new state of the shooter
     */
    public void setState(States state) {
        // System.out.println(state + Thread.currentThread().getStackTrace()[2].getClassName());
        if (state.equals(States.READY_TO_FIRE)) {
            firing.activate();
        } else {
            firing.cancel();
        }
        shooterState = state;
    }

    public States getState() {
        return shooterState;
    }

    /**
     * Sets the motor speed from -1 < value < 1.
     *
     * @param power speed to set the motor to
     */
    public void setMotorPower(double power) {
        shooterMotorMain.set(power);
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
        volts = Functions.clampDouble(volts, 12, -12);
        shooterMotorMain.setVoltage(volts);
    }

    /**
     * Sets the motor to run to a target speed in RPM.
     *
     * @param speed The target speed in RPM.
     */
    public void setMotorTargetSpeed(double speed) {
        speed = Functions.clampDouble(speed, MAX_RPM, -MAX_RPM);
        setPoint = speed;
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
    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    public double calculateVoltageFromPid(double velocity) {
        velocity /= 30;
        return Functions.clampDouble(pidDum.calculate(getShooterRPM(), velocity) + ffDum.calculate(velocity), 12, -12);
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

    public String getShooterStateAsText() {
        return shooterState.toString();
    }

    @Override
    public String getTestName() {
        return "Shooter";
    }

    @Override
    public ArrayList<CANSparkMax> getMotors() {
        ArrayList<CANSparkMax> result = new ArrayList<CANSparkMax>();
        result.add(shooterMotorMain);
        return result;
    }

    @Override
    public HashMap<String, Lemonlight> getLimelights() {
        HashMap<String, Lemonlight> result = new HashMap<String, Lemonlight>();
        result.put("limelight-target", targetingLimelight);
        return result;
    }

    @Override
    public double getMotorTestSpeed() {
        return 0.15;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        // //builder.addDoubleProperty("encoderValue", this::getEncoderValue, null);
        builder.addDoubleProperty("shooterRPM", this::getShooterRPM, null);
        builder.addDoubleProperty("shooterSetpoint", () -> setPoint, null);
        builder.addBooleanProperty("hoodPosition", this::getHoodPos, null);
        // builder.addStringProperty("shooterState", this::getShooterStateAsText, null);
    }
}
