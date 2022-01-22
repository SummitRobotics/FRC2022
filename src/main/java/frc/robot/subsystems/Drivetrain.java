package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Ports;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Subsystem to control the drivetrain of the robot
 */
public class Drivetrain extends SubsystemBase {

    // TODO re tune/calculate all these
    public static final double LOW_P = 2.29,
            LOW_I = 0.0,
            LOW_D = 0.0,
            LOW_KS = 0.177,
            LOW_KV = 1.55,
            LOW_KA = 0.133,
            HIGH_P = 2.85,
            HIGH_I = 0.0,
            HIGH_D = 0.0,
            HIGH_KS = 0.211,
            HIGH_KV = 0.734,
            HIGH_KA = 0.141,
            HIGH_GEAR_RATIO = 9.1,
            LOW_GEAR_RATIO = 19.65,
            WHEEL_RADIUS_IN_METERS = 0.0762,
            WHEEL_CIRCUMFERENCE_IN_METERS = (2 * WHEEL_RADIUS_IN_METERS) * Math.PI,
            MAX_OUTPUT_VOLTAGE = 11,
            DRIVE_WIDTH = 0.7112;

    // left motors
    private CANSparkMax left = new CANSparkMax(Ports.LEFT_DRIVE_3, MotorType.kBrushless);
    private CANSparkMax leftMiddle = new CANSparkMax(Ports.LEFT_DRIVE_2, MotorType.kBrushless);
    private CANSparkMax leftBack = new CANSparkMax(Ports.LEFT_DRIVE_1, MotorType.kBrushless);

    // right motors
    private CANSparkMax right = new CANSparkMax(Ports.RIGHT_DRIVE_3, MotorType.kBrushless);
    private CANSparkMax rightMiddle = new CANSparkMax(Ports.RIGHT_DRIVE_2, MotorType.kBrushless);
    private CANSparkMax rightBack = new CANSparkMax(Ports.RIGHT_DRIVE_1, MotorType.kBrushless);

    // pid controllers
    private SparkMaxPIDController leftPID = left.getPIDController();
    private SparkMaxPIDController rightPID = right.getPIDController();

    // encoders
    private RelativeEncoder leftEncoder = left.getEncoder();
    private RelativeEncoder rightEncoder = right.getEncoder();

    private DifferentialDriveOdometry odometry;

    private BooleanSupplier shift;
    private AHRS gyro;

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

    /**
     * i am in PAIN wow this is BAD
     *
     * @param gyro       odimetry is bad
     * @param shiftState shifting was a mistake just be glad i am trying to limit my sins and am not
     *                   passing in the whole shift object in and using fake callbacks from hell
     */
    public Drivetrain(AHRS gyro, BooleanSupplier shiftState) {

        this.gyro = gyro;
        this.shift = shiftState;

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        // tells other two motors to follow the first
        leftMiddle.follow(left);
        leftBack.follow(left);

        rightMiddle.follow(right);
        rightBack.follow(right);

        // inverts right side
        left.setInverted(true);
        right.setInverted(false);

        // sets pid values
        zeroEncoders();

        // pid for position
        leftPID.setP(0.05);
        leftPID.setI(0);
        leftPID.setD(0);
        leftPID.setOutputRange(-.25, .25);

        rightPID.setP(0.05);
        rightPID.setI(0);
        rightPID.setD(0);
        rightPID.setOutputRange(-.25, .25);

        // pid for velocity
        leftPID.setP(0.000278, 2);
        leftPID.setI(0, 2);
        leftPID.setD(0.0001, 2);

        rightPID.setP(0.000278, 2);
        rightPID.setI(0, 2);
        rightPID.setD(0.0001, 2);

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
     * Sets the power of the left side of the drivetrain
     *
     * @param power -1 - 1
     */
    public void setLeftMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        synchronized (left) {
            left.set(power);
        }
    }

    /**
     * Sets the power of the right side of the drivetrain
     *
     * @param power -1 - 1
     */
    public void setRightMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        synchronized (right) {
            right.set(power);
        }
    }

    public void setLeftMotorVolts(double volts) {
        synchronized (left) {
            left.setVoltage(volts);
        }
    }

    public void setRightMotorVolts(double volts) {
        synchronized (right) {
            right.setVoltage(volts);
        }
    }

    /**
     * Sets the voltage of the motors
     *
     * @param left  the right motor voltage
     * @param right the left motor voltage
     */
    public void setMotorVolts(double left, double right) {
        // System.out.println(String.format("left is: %f, right is %f", left, right));
        setRightMotorVolts(right);
        setLeftMotorVolts(left);
    }

    /**
     * Sets the motor power based on desired meters per second
     *
     * @param leftMS  the left motor MPS
     * @param rightMS the right motor MPS
     */
    public void setMotorTargetSpeed(double leftMS, double rightMS) {
        leftPID.setReference(MPStoRPM(leftMS), ControlType.kVelocity, 2);
        rightPID.setReference(MPStoRPM(rightMS), ControlType.kVelocity, 2);
    }

    /**
     * Converts robot meters per second into motor rotations per minute
     *
     * @param input the MPS to convert
     * @return the corresponding RPM
     */
    public double MPStoRPM(double input) {
        double out = input / WHEEL_RADIUS_IN_METERS;
        out = out * 60;
        out = out * (2 * Math.PI);
        out = out * HIGH_GEAR_RATIO;
        out /= 39.4784176044;
        return out;
    }

    /**
     * Sets the target position of the left side of the drivetrain
     *
     * @param position the target position in terms of motor rotations
     */
    public synchronized void setLeftMotorTarget(double position) {
        leftPID.setReference(position, ControlType.kPosition);
    }

    /**
     * Sets the target position of the right side of the drivetrain
     *
     * @param position the target position in terms of motor rotations
     */
    public synchronized void setRightMotorTarget(double position) {
        rightPID.setReference(position, ControlType.kPosition);
    }

    /**
     * The position you want the left side to register when it is in the position it is currently in
     *
     * @param position the position for the encoder to register in rotations
     */
    public synchronized void setLeftEncoder(double position) {
        leftEncoder.setPosition(position);
    }

    /**
     * The position you want the right side to register when it is in the position it is currently in
     *
     * @param position the position for the encoder to register in rotations
     */
    public synchronized void setRightEncoder(double position) {
        rightEncoder.setPosition(position);
    }

    public synchronized void zeroEncoders() {
        setRightEncoder(0);
        setLeftEncoder(0);
    }

    /**
     * Returns the current position of right side of the drivetrain
     *
     * @return position of motor in rotations
     */
    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }

    /**
     * Returns the current position of right side of the drivetrain
     *
     * @return position of motor in rotations
     */
    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getLeftRPM() {
        return leftEncoder.getVelocity();
    }

    public double getRightRPM() {
        return rightEncoder.getVelocity();
    }

    /**
     * @return the total distance in meters the side as travled sense the last reset
     */
    public double getLeftDistance() {
        if (shift.getAsBoolean()) {
            return (getLeftEncoderPosition() / HIGH_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS;
        } else {
            return (getLeftEncoderPosition() / LOW_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS;
        }
    }

    /**
     * @return the total distance in meters the side as travled sense the last reset
     */
    public synchronized double getRightDistance() {
        if (shift.getAsBoolean()) {
            return (getRightEncoderPosition() / HIGH_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS;
        } else {
            return (getRightEncoderPosition() / LOW_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS;
        }
    }

    /**
     * @return the linear speed of the side in meters per second
     */
    public synchronized double getLeftSpeed() {
        if (shift.getAsBoolean()) {
            return convertRpmToMetersPerSecond((getLeftRPM() / HIGH_GEAR_RATIO));
        } else {
            return convertRpmToMetersPerSecond((getLeftRPM() / LOW_GEAR_RATIO));
        }
    }

    /**
     * @return the linear speed of the side in meters per second
     */
    public synchronized double getRightSpeed() {
        if (shift.getAsBoolean()) {
            return convertRpmToMetersPerSecond((getRightRPM() / HIGH_GEAR_RATIO));
        } else {
            return convertRpmToMetersPerSecond((getRightRPM() / LOW_GEAR_RATIO));
        }
    }

    // could things be good for once please
    private double convertRpmToMetersPerSecond(double RPM) {
        return ((RPM / 60) * (2 * Math.PI)) * WHEEL_RADIUS_IN_METERS;
    }

    /**
     * Sets the rate at witch the motors ramp up and down in open loop control mode
     *
     * @param rate time in seconds to go from 0 to full power
     */
    public void setOpenRampRate(double rate) {
        left.setOpenLoopRampRate(rate);
        right.setOpenLoopRampRate(rate);
    }

    /**
     * Sets the rate at which the motors ramp up and down in closed loop control mode
     *
     * @param rate time in seconds to go from 0 to full power
     */
    public void setClosedRampRate(double rate) {
        left.setClosedLoopRampRate(rate);
        right.setClosedLoopRampRate(rate);
    }

    /**
     * Stops the motors
     */
    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }

    public synchronized DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
    }

    /**
     * sets the curent pos and RESETS ENCODERS TO 0
     *
     * @param pose the new pose
     */
    public synchronized void setPose(Pose2d pose) {
        zeroEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public synchronized Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * gets the right pid values for the curent shift state
     *
     * @return double array of p,i,d
     */
    public double[] getPid() {
        if (shift.getAsBoolean()) {
            double[] out = {HIGH_P, HIGH_I, HIGH_D};
            return out;

        } else {
            double[] out = {LOW_P, LOW_I, LOW_D};
            return out;
        }
    }

    public boolean getShift() {
        return shift.getAsBoolean();
    }

    public SimpleMotorFeedforward getFeedFoward() {
        if (getShift()) {
            return HighFeedFoward;
        } else {
            return LowFeedFoward;
        }
    }

    public DifferentialDriveVoltageConstraint getVoltageConstraint() {
        if (getShift()) {
            return HighVoltageConstraint;
        } else {
            return LowVoltageConstraint;
        }
    }

    public synchronized void updateOdometry() {
        odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    }

    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();
        // System.out.println(MPStoRPM(getRightSpeed()));
        // System.out.println(rightEncoder.getVelocity());
        // System.out.println("------------------------");
    }
}
