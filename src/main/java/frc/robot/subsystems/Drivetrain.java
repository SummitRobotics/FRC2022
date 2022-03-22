package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the drivetrain of the robot.
 */
public class Drivetrain extends SubsystemBase {

    // TODO re tune/calculate all these
    public static final double 
        LOW_P = 0.0,
        LOW_I = 0.0,
        LOW_D = 0.0,
        LOW_KS = 0.0,
        LOW_KV = 0.0,
        LOW_KA = 0.0,
        HIGH_P = 14.301,
        HIGH_I = 0.0,
        HIGH_D = 581.73,
        HIGH_KS = 0.18976,
        HIGH_KV = 1.9778,
        HIGH_KA = 0.16066,
        HIGH_GEAR_RATIO = 5.088,
        LOW_GEAR_RATIO = 11.022,
        WHEEL_RADIUS_IN_METERS = 0.0508,
        WHEEL_CIRCUMFRENCE_IN_METERS = (2 * WHEEL_RADIUS_IN_METERS) * Math.PI,
        MAX_OUTPUT_VOLTAGE = 11,
        DRIVE_WIDTH = 0.6858;


    // left motors
    private final CANSparkMax left =
        new CANSparkMax(Ports.LEFT_DRIVE_3, MotorType.kBrushless);

    private final CANSparkMax leftMiddle =
        new CANSparkMax(Ports.LEFT_DRIVE_2, MotorType.kBrushless);

    private final CANSparkMax leftBack =
        new CANSparkMax(Ports.LEFT_DRIVE_1, MotorType.kBrushless);


    // right motors
    private final CANSparkMax right =
        new CANSparkMax(Ports.RIGHT_DRIVE_3, MotorType.kBrushless);

    private final CANSparkMax rightMiddle =
        new CANSparkMax(Ports.RIGHT_DRIVE_2, MotorType.kBrushless);

    private final CANSparkMax rightBack =
        new CANSparkMax(Ports.RIGHT_DRIVE_1, MotorType.kBrushless);


    // pid controllers
    private final SparkMaxPIDController leftPID = left.getPIDController();
    private final SparkMaxPIDController leftMiddlePID = leftMiddle.getPIDController();
    private final SparkMaxPIDController leftBackPID = leftBack.getPIDController();
    private final SparkMaxPIDController rightPID = right.getPIDController();
    private final SparkMaxPIDController rightMiddlePID = rightMiddle.getPIDController();
    private final SparkMaxPIDController rightBackPID = rightBack.getPIDController();

    // encoders
    private final RelativeEncoder leftEncoder = left.getEncoder();
    private final RelativeEncoder leftMiddleEncoder = leftMiddle.getEncoder();
    private final RelativeEncoder leftBackEncoder = leftBack.getEncoder();
    private final RelativeEncoder rightEncoder = right.getEncoder();
    private final RelativeEncoder rightMiddleEncoder = rightMiddle.getEncoder();
    private final RelativeEncoder rightBackEncoder = rightBack.getEncoder();

    private final DifferentialDriveOdometry odometry;

    private final AHRS gyro;

    public static DifferentialDriveKinematics DriveKinimatics =
        new DifferentialDriveKinematics(DRIVE_WIDTH);

    public static SimpleMotorFeedforward HighFeedFoward =
        new SimpleMotorFeedforward(HIGH_KS, HIGH_KV, HIGH_KA);

    public static SimpleMotorFeedforward LowFeedFoward =
        new SimpleMotorFeedforward(LOW_KS, LOW_KV, LOW_KA);

    public static DifferentialDriveVoltageConstraint HighVoltageConstraint =
        new DifferentialDriveVoltageConstraint(HighFeedFoward, DriveKinimatics, MAX_OUTPUT_VOLTAGE);

    public static DifferentialDriveVoltageConstraint LowVoltageConstraint =
        new DifferentialDriveVoltageConstraint(LowFeedFoward, DriveKinimatics, MAX_OUTPUT_VOLTAGE);

    private final Solenoid shift;

    private boolean oldShift;

    //for making robot distance consistent across shifts
    private double leftDistanceAcum = 0;
    private double rightDistanceAcum = 0;

    private final Timer odometryTime = new Timer();

    private final Field2d f2d;

    private LEDCall lowGear = new LEDCall(LEDPriorities.LOW_GEAR, LEDRange.All).sine(Colors.RED);
    
    /**
     * i am in PAIN wow this is BAD.
     *
     * @param gyro       odimetry is bad
     */
    public Drivetrain(AHRS gyro) {
        this.gyro = gyro;

        shift = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, Ports.SHIFT_SOLENOID);

        odometryTime.reset();
        odometryTime.start();


        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        f2d = new Field2d();

        // tells other two motors to follow the first
        

        // inverts right side
        left.setInverted(true);
        leftMiddle.setInverted(true);
        leftBack.setInverted(true);
        right.setInverted(false);
        rightMiddle.setInverted(false);
        rightBack.setInverted(false);

        // sets pid values
        zeroDistance();

        // pid for position
        leftPID.setP(14.301);
        leftPID.setI(0);
        leftPID.setD(581.73);
        leftPID.setOutputRange(-.25, .25);

        rightPID.setP(14.301);
        rightPID.setI(0);
        rightPID.setD(581.73);
        rightPID.setOutputRange(-.25, .25);

        // pid for velocity
        leftPID.setP(0.00012245, 2);
        leftPID.setI(0, 2);
        leftPID.setD(0.0, 2);

        rightPID.setP(0.00012245, 2);
        rightPID.setI(0, 2);
        rightPID.setD(0.0, 2);
        
        // pid for position
        leftMiddlePID.setP(14.301);
        leftMiddlePID.setI(0);
        leftMiddlePID.setD(581.73);
        leftMiddlePID.setOutputRange(-.25, .25);

        rightMiddlePID.setP(14.301);
        rightMiddlePID.setI(0);
        rightMiddlePID.setD(581.73);
        rightMiddlePID.setOutputRange(-.25, .25);

        // pid for velocity
        leftMiddlePID.setP(0.00012245, 2);
        leftMiddlePID.setI(0, 2);
        leftMiddlePID.setD(0.0, 2);

        rightMiddlePID.setP(0.00012245, 2);
        rightMiddlePID.setI(0, 2);
        rightMiddlePID.setD(0.0, 2);
        
        // pid for position
        leftBackPID.setP(14.301);
        leftBackPID.setI(0);
        leftBackPID.setD(581.73);
        leftBackPID.setOutputRange(-.25, .25);

        rightBackPID.setP(14.301);
        rightBackPID.setI(0);
        rightBackPID.setD(581.73);
        rightBackPID.setOutputRange(-.25, .25);

        // pid for velocity
        leftBackPID.setP(0.00012245, 2);
        leftBackPID.setI(0, 2);
        leftBackPID.setD(0.0, 2);

        rightBackPID.setP(0.00012245, 2);
        rightBackPID.setI(0, 2);
        rightBackPID.setD(0.0, 2);

        left.disableVoltageCompensation();
        right.disableVoltageCompensation();

        leftMiddle.disableVoltageCompensation();
        rightMiddle.disableVoltageCompensation();

        leftBack.disableVoltageCompensation();
        rightBack.disableVoltageCompensation();

        setClosedRampRate(0);
        setOpenRampRate(0);

        left.setSmartCurrentLimit(40);
        leftMiddle.setSmartCurrentLimit(40);
        leftBack.setSmartCurrentLimit(40);

        right.setSmartCurrentLimit(40);
        rightMiddle.setSmartCurrentLimit(40);
        rightBack.setSmartCurrentLimit(40);

        left.setIdleMode(IdleMode.kBrake);
        leftMiddle.setIdleMode(IdleMode.kBrake);
        leftBack.setIdleMode(IdleMode.kBrake);

        right.setIdleMode(IdleMode.kBrake);
        rightMiddle.setIdleMode(IdleMode.kBrake);
        rightBack.setIdleMode(IdleMode.kBrake);

    }


    /**
     * Shifts the robot into high gear.
     */
    public void highGear() {
        lowGear.cancel();
        oldShift = false;
        updateDistanceAcum();
        shift.set(false);
    }

    /**
     * Shifts the robot into low gear.
     */
    public void lowGear() {
        lowGear.activate();
        oldShift = true;
        updateDistanceAcum();
        shift.set(true);
    }

    /**
     * Toggles the shift state.
     */
    public void toggleShift() {
        //System.out.println("shift call");
        if (oldShift) {
            highGear();
        } else {
            lowGear();
        }
    }

    /**
     * Updates the distace acumulator.
     */
    private void updateDistanceAcum() {
        leftDistanceAcum += getLeftDistance();
        rightDistanceAcum += getRightDistance();
        zeroEncoders();
    }

    /**
     * Gets the shift state.
     *
     * @return the shift state where true is high and false is low
     */
    public boolean getShift() {
        return oldShift;
    }

    /**
     * Sets the power of the left side of the drivetrain.
     *
     * @param power -1 - 1
     */
    public void setLeftMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        left.set(power);
        leftMiddle.set(power);
        leftBack.set(power);
    }

    /**
     * Sets the power of the right side of the drivetrain.
     *
     * @param power -1 - 1
     */
    public void setRightMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        right.set(power);
        rightMiddle.set(power);
        rightBack.set(power);
    }

    /**
     * Sets the power of both sides of the drivetrain.
     *
     * @param power The power, between -1 and 1
     */
    public synchronized void setBothMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        left.set(power);
        leftMiddle.set(power);
        leftBack.set(power);
        right.set(power);
        rightMiddle.set(power);
        rightBack.set(power);
    }

    /**
     * Sets the voltage to the left motors.
     *
     * @param volts Amount of volts to send to left motors.
     */
    public void setLeftMotorVolts(double volts) {
        left.setVoltage(volts);
        leftMiddle.setVoltage(volts);
        leftBack.setVoltage(volts);
    }

    /**
     * Sets the voltage to the right motors.
     *
     * @param volts Amount of volts to send to the right motors.
     */
    public void setRightMotorVolts(double volts) {
        right.setVoltage(volts);
        rightMiddle.setVoltage(volts);
        rightBack.setVoltage(volts);
    }

    /**
     * Sets the voltage of the motors.
     *
     * @param left  the right motor voltage
     * @param right the left motor voltage
     */
    public void setMotorVolts(double left, double right) {
        //System.out.println(String.format("left is: %f, right is %f", left, right));
        setRightMotorVolts(right);
        setLeftMotorVolts(left);
    }

    /**
     * Sets the motor power based on desired meters per second.
     *
     * @param leftMS  the left motor MPS
     * @param rightMS the right motor MPS
     */
    public void setMotorTargetSpeed(double leftMS, double rightMS) {
        //leftPID.setFF(getFeedForward().calculate(leftMS));
        //rightPID.setFF(getFeedForward().calculate(rightMS));
        leftPID.setReference(convertMPStoRPM(leftMS), ControlType.kVelocity, 2);
        leftMiddlePID.setReference(convertMPStoRPM(leftMS), ControlType.kVelocity, 2);
        leftBackPID.setReference(convertMPStoRPM(leftMS), ControlType.kVelocity, 2);
        rightPID.setReference(convertMPStoRPM(rightMS), ControlType.kVelocity, 2);
        rightMiddlePID.setReference(convertMPStoRPM(rightMS), ControlType.kVelocity, 2);
        rightBackPID.setReference(convertMPStoRPM(rightMS), ControlType.kVelocity, 2);
        
    }

    /**
     * Converts robot meters per second into motor rotations per minute.
     *
     * @param input the MPS to convert
     * @return the corresponding RPM
     */
    public double convertMPStoRPM(double input) {
        double out = input / WHEEL_RADIUS_IN_METERS;
        out *= 60;
        out *= (2 * Math.PI);
        out *= HIGH_GEAR_RATIO;
        out /= 39.4784176044;
        return out;
    }

    /**
     * Sets the target position of the left side of the drivetrain.
     *
     * @param position the target position in terms of motor rotations
     */
    public synchronized void setLeftMotorTarget(double position) {
        leftPID.setReference(position, ControlType.kPosition);
        leftMiddlePID.setReference(position, ControlType.kPosition);
        leftBackPID.setReference(position, ControlType.kPosition);
    }

    /**
     * Sets the target position of the right side of the drivetrain.
     *
     * @param position the target position in terms of motor rotations
     */
    public synchronized void setRightMotorTarget(double position) {
        rightPID.setReference(position, ControlType.kPosition);
        rightMiddlePID.setReference(position, ControlType.kPosition);
        rightBackPID.setReference(position, ControlType.kPosition);
    }

    /**
     * Convert distance to encoder values.
     * TODO: STILL NEED TO TEST THIS
     *
     * @param dist the distance in meters.
     * @return The converstion to encover values.
     */
    public double distToEncoder(double dist) {
        if (getShift()) {
            return (dist / WHEEL_CIRCUMFRENCE_IN_METERS) * HIGH_GEAR_RATIO;
        } else {
            return (dist / WHEEL_CIRCUMFRENCE_IN_METERS) * LOW_GEAR_RATIO;
        }
    }
    
    /**
     * The position you want the left side to register.
     * When it is in the position it is currently in
     *
     * @param position the position for the encoder to register in rotations
     */
    public synchronized void setLeftEncoder(double position) {
        leftEncoder.setPosition(position);
        leftMiddleEncoder.setPosition(position);
        leftBackEncoder.setPosition(position);
    }

    /**
     * The position you want the right side to register.
     * When it is in the position it is currently in
     *
     * @param position the position for the encoder to register in rotations
     */
    public synchronized void setRightEncoder(double position) {
        rightEncoder.setPosition(position);
        rightMiddleEncoder.setPosition(position);
        rightBackEncoder.setPosition(position);
        
    }

    /**
     * Zeros the raw encoder values (PROBABLY NOT WHAT YOU WANT).
     *
     * @apiNote This zeros the raw encoder values. This is provavly not what you want to do.
     *     Instead use zeroEncoders().
     */
    private synchronized void zeroEncoders() {
        setRightEncoder(0);
        setLeftEncoder(0);
    }

    /**
     * zeros the enocders and encoder acum. 
     *
     * @apiNote is probably what you want
     */
    public synchronized void zeroDistance() {
        leftDistanceAcum = 0;
        rightDistanceAcum = 0;
        zeroEncoders();
    }

    /**
     * Returns the current position of right side of the drivetrain.
     *
     * @return position of motor in rotations
     */
    public double getRightEncoderPosition() {
        return (rightEncoder.getPosition() + rightMiddleEncoder.getPosition() + rightBackEncoder.getPosition()) / 3;
    }

    /**
     * Returns the current position of right side of the drivetrain.
     *
     * @return position of motor in rotations
     */
    public double getLeftEncoderPosition() {
        return (leftEncoder.getPosition() + leftMiddleEncoder.getPosition() + leftBackEncoder.getPosition()) / 3;
    }

    public double getLeftRPM() {
        return (leftEncoder.getVelocity() + leftMiddleEncoder.getVelocity() + leftBackEncoder.getVelocity()) / 3;
    }

    public double getRightRPM() {
        return (leftEncoder.getVelocity() + leftMiddleEncoder.getVelocity() + leftBackEncoder.getVelocity()) / 3;
    }

    /**
     * Gets the total distance the left side has traveled since its last reset.
     *
     * @return the total distance in meters the side as travled sense the last reset
     */
    public double getLeftDistance() {
        if (getShift()) {
            return ((getLeftEncoderPosition() / HIGH_GEAR_RATIO) * WHEEL_CIRCUMFRENCE_IN_METERS)
                + leftDistanceAcum;
        } else {
            return ((getLeftEncoderPosition() / LOW_GEAR_RATIO) * WHEEL_CIRCUMFRENCE_IN_METERS)
                + leftDistanceAcum;
        }
    }

    /**
     * Gets the total distance the left side has traveled since its last reset.
     *
     * @return the total distance in meters the side as travled sense the last reset
     */
    public synchronized double getRightDistance() {
        if (getShift()) {
            return ((getRightEncoderPosition() / HIGH_GEAR_RATIO) * WHEEL_CIRCUMFRENCE_IN_METERS)
                + rightDistanceAcum;
        } else {
            return ((getRightEncoderPosition() / LOW_GEAR_RATIO) * WHEEL_CIRCUMFRENCE_IN_METERS)
                + rightDistanceAcum;
        }
    }

    /**
     * Gets the linear speed of the left side in m/s.
     *
     * @return the linear speed of the side in meters per second
     */
    public synchronized double getLeftSpeed() {
        if (getShift()) {
            return convertRpmToMetersPerSecond((getLeftRPM() / HIGH_GEAR_RATIO));
        } else {
            return convertRpmToMetersPerSecond((getLeftRPM() / LOW_GEAR_RATIO));
        }
    }

    /**
     * Gets the linear speed of the right side in m/s.
     *
     * @return the linear speed of the side in meters per second
     */
    public synchronized double getRightSpeed() {
        if (getShift()) {
            return convertRpmToMetersPerSecond((getRightRPM() / HIGH_GEAR_RATIO));
        } else {
            return convertRpmToMetersPerSecond((getRightRPM() / LOW_GEAR_RATIO));
        }
    }

    // could things be good for once please
    private double convertRpmToMetersPerSecond(double rpm) {
        return ((rpm / 60) * (2 * Math.PI)) * WHEEL_RADIUS_IN_METERS;
    }

    /**
     * Sets the rate at which the motors ramp up and down in open loop control mode.
     *
     * @param rate time in seconds to go from 0 to full power
     */
    public void setOpenRampRate(double rate) {
        left.setOpenLoopRampRate(rate);
        leftMiddle.setOpenLoopRampRate(rate);
        leftBack.setOpenLoopRampRate(rate);
        right.setOpenLoopRampRate(rate);
        rightMiddle.setOpenLoopRampRate(rate);
        rightBack.setOpenLoopRampRate(rate);
    }

    /**
     * Sets the rate at which the motors ramp up and down in closed loop control mode.
     *
     * @param rate time in seconds to go from 0 to full power
     */
    public void setClosedRampRate(double rate) {
        left.setClosedLoopRampRate(rate);
        leftMiddle.setClosedLoopRampRate(rate);
        leftBack.setClosedLoopRampRate(rate);
        right.setClosedLoopRampRate(rate);
        rightMiddle.setClosedLoopRampRate(rate);
        rightBack.setClosedLoopRampRate(rate);
    }

    public double getLeftMotorCurrent() {
        return (left.getOutputCurrent() + leftMiddle.getOutputCurrent() + leftBack.getOutputCurrent()) / 3;
    }

    public double getRightMotorCurrent() {
        return (right.getOutputCurrent() + rightMiddle.getOutputCurrent() + rightBack.getOutputCurrent()) / 3;
    }

    /**
     * Stops the motors.
     */
    public void stop() {
        left.stopMotor();
        leftMiddle.stopMotor();
        leftBack.stopMotor();
        right.stopMotor();
        rightMiddle.stopMotor();
        rightBack.stopMotor();
    }

    public synchronized DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
    }

    /**
     * sets the curent pos and RESETS ENCODERS TO 0.
     *
     * @param pose the new pose
     */
    public synchronized void setPose(Pose2d pose) {
        zeroDistance();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public synchronized Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * gets the right pid values for the curent shift state.
     *
     * @return double array of p,i,d
     */
    public double[] getPid() {
        if (getShift()) {
            double[] out = { HIGH_P, HIGH_I, HIGH_D };
            return out;

        } else {
            double[] out = { LOW_P, LOW_I, LOW_D };
            return out;
        }
    }

    /**
     * Gets the feed forward.
     *
     * @return The motors feed forward.
     */
    public SimpleMotorFeedforward getFeedForward() {
        if (getShift()) {
            return HighFeedFoward;
        } else {
            return LowFeedFoward;
        }
    }
    
    /**
     * Gets the current voltage constant placed on the drivetrain.
     *
     * @return the current voltage constraint.
     */
    public DifferentialDriveVoltageConstraint getVoltageConstraint() {
        if (getShift()) {
            return HighVoltageConstraint;
        } else {
            return LowVoltageConstraint;
        }
    }

    //TODO run a check so we dont update at a unnessaryly fast rate
    //TODO CONT beacuse its being updated by periodic and followSpline

    /**
     * Updates odometry.
     * It only updates at a rate of 500hz maximum.
     */
    public synchronized void updateOdometry() {
        //prevemts unnessarly fast updates to the odemetry (2 ms)
        if (odometryTime.get() > 0.002) {
            odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
            odometryTime.reset();
        }
        f2d.setRobotPose(odometry.getPoseMeters());
    }

    public double getRotation() {
        return gyro.getRotation2d().getDegrees();
    }


    /**
     * Method that runs once per scheduler cycle.
     */
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();
        // System.out.println(MPStoRPM(getRightSpeed()));
        // System.out.println(rightEncoder.getVelocity());
        // System.out.println("------------------------");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //builder.setSmartDashboardType("Drivetrain");

        //builder.addDoubleProperty("leftDistance", this::getLeftDistance, null);
        //builder.addDoubleProperty("leftEncoder", this::getLeftEncoderPosition, null);
        //builder.addDoubleProperty("leftRPM", this::getLeftRPM, null);
        //builder.addDoubleProperty("leftSpeed", this::getLeftSpeed, null);

        //builder.addDoubleProperty("rightDistance", this::getRightDistance, null);
        //builder.addDoubleProperty("rightEncoder", this::getRightEncoderPosition, null);
        //builder.addDoubleProperty("rightRPM", this::getRightRPM, null);
        //builder.addDoubleProperty("rightSpeed", this::getRightSpeed, null);
        //builder.addDoubleProperty("rotation", this::getRotation, null);
        // f2d.initSendable(builder);

        // builder.addBooleanProperty("shifterStatus", this::getShift, null);
        //builder.addDoubleArrayProperty("pidValues", this::getPid, null);
    }
}
