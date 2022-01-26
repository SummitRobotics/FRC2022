package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
     * @param power The power of the front motor (between 1 and -1).
     */
    public void setFrontMotorPower(double power) {
        front.set(power);
    }

    /**
     * Sets the power of the back motor.
     *
     * @param power The power of the back motor (between 1 and -1).
     */
    public void setBackMotorPower(double power) {
        back.set(power);
    }

    /**
     * Sets the power of both motors.
     */
    public void setMotorPower(double power) {
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
     * @param position The front encoder's position in rotations.
     */
    public void setFrontEncoder(double position) {
        frontEncoder.setPosition(position);
    }

    /**
     * Manually sets the back encoder's position (in rotations).
     *
     * @param position The back encoder's position in rotations.
     */
    public void setBackEncoder(double position) {
        backEncoder.setPosition(position);
    }

    /**
     * Gets the speed of the front motor (in RPM).
     *
     * @return speed The speed of the front motor in RPM.
     */
    public double getFrontRPM() {
        return frontEncoder.getVelocity();
    }

    /**
     * Gets the speed of the back motor (in RPM).
     *
     * @return speed The speed of the back motor in RPM.
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
     * @param rate How quickly the motors accelerate and decelerate, measured in the number of
     *      seconds it takes for the motor to ramp up to full speed.
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
