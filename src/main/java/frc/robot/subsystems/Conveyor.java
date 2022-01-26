package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Ports;

public class Conveyor extends SubsystemBase {

    private final CANSparkMax front = new CANSparkMax(Ports.FRONT_CONVEYOR, MotorType.kBrushless);
    private final CANSparkMax back = new CANSparkMax(Ports.BACK_CONVEYOR, MotorType.kBrushless);

    private final RelativeEncoder frontEncoder = front.getEncoder();
    private final RelativeEncoder backEncoder = back.getEncoder();

    public Conveyor() {}
    
    public void setFrontMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        front.set(power);
    }

    public void setBackMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        back.set(power);
    }

    public void setMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        front.set(power);
        back.set(power);
    }
    
    public double getFrontEncoderPosition() {
        return frontEncoder.getPosition();
    }

    public double getBackEncoderPosition() {
        return backEncoder.getPosition();
    }

    public void setFrontEncoder(double position) {
        frontEncoder.setPosition(position);
    }

    public void setBackEncoder(double position) {
        backEncoder.setPosition(position);
    }

    public double getFrontRPM() {
        return frontEncoder.getVelocity();
    }

    public double getBackRPM() {
        return backEncoder.getVelocity();
    }

    public void zeroEncoders() {
        setFrontEncoder(0);
        setBackEncoder(0);
    }

    public void setClosedRampRate(double rate) {
        front.setClosedLoopRampRate(rate);
        back.setClosedLoopRampRate(rate);
    }

    public void stop() {
        front.stopMotor();
        back.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Conveyor");
    }
}
