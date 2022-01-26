package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the conveyor of the robot.
 */
public class Conveyor extends SubsystemBase {

    // motors
    private final CANSparkMax front = new CANSparkMax(Ports.FRONT_CONVEYOR, MotorType.kBrushless);
    private final CANSparkMax back = new CANSparkMax(Ports.BACK_CONVEYOR, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder frontEncoder = front.getEncoder();
    private final RelativeEncoder backEncoder = back.getEncoder();

    public Conveyor() {
        zeroEncoders();
    }

    /**
     * Sets the power of the front motor.
     * 
     * @param power
     */
    public void setFrontMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        front.set(power);
    }

    /**
     * Sets the power of the back motor.
     * 
     * @param power
     */
    public void setBackMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        back.set(power);
    }

    /**
     * Sets the power of both motors.
     */
    public void setMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        front.set(power);
        back.set(power);
    }

    /**
     * Gets the encoder position of the front motor (in rotations).
     * 
     * @return position
     */
    public double getFrontEncoderPosition() {
        return frontEncoder.getPosition();
    }

    /**
     * Gets the encoder position of the back motor (in rotations).
     * 
     * @return position
     */
    public double getBackEncoderPosition() {
        return backEncoder.getPosition();
    }

    /**
     * Manually sets the front encoder's position (in rotations).
     * 
     * @param position
     */
    public void setFrontEncoder(double position) {
        frontEncoder.setPosition(position);
    }

    /**
     * Manually sets the back encoder's position (in rotations).
     * 
     * @param position
     */
    public void setBackEncoder(double position) {
        backEncoder.setPosition(position);
    }

    /**
     * Gets the speed of the front motor (in RPM).
     * 
     * @return speed
     */
    public double getFrontRPM() {
        return frontEncoder.getVelocity();
    }

    /**
     * Gets the speed of the back motor (in RPM).
     * 
     * @return speed
     */
    public double getBackRPM() {
        return backEncoder.getVelocity();
    }

    /**
     * Resets the encoder values to 0.
     */
    public void zeroEncoders() {
        setFrontEncoder(0);
        setBackEncoder(0);
    }

    /**
     * Sets how quickly the motors accelerate and decelerate. This is measured in
     * the number of seconds it takes for the motor to ramp up to full speed.
     * 
     * @param rate
     */
    public void setOpenRampRate(double rate) {
        front.setOpenLoopRampRate(rate);
        back.setOpenLoopRampRate(rate);
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        front.stopMotor();
        back.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("conveyor");
        builder.addDoubleProperty("front_motor_position", this::getFrontEncoderPosition, null);
        builder.addDoubleProperty("back_motor_position", this::getBackEncoderPosition, null);
        builder.addDoubleProperty("front_motor_speed", this::getFrontRPM, null);
        builder.addDoubleProperty("back_motor_speed", this::getBackRPM, null);
    }
}
