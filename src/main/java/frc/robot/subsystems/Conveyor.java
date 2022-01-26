package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Ports;

public class Conveyor extends SubsystemBase {

    private final CANSparkMax front = new CANSparkMax(Ports.FRONT_CONVEYOR, MotorType.kBrushless);
    private final CANSparkMax back = new CANSparkMax(Ports.BACK_CONVEYOR, MotorType.kBrushless);

    private final RelativeEncoder frontEncoder = front.getEncoder();
    private final RelativeEncoder backEncoder = back.getEncoder();

    public enum States {
        SHOOT,
        INTAKE,
        OFF
    }

    public Conveyor() {}
    
    public void setMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        synchronized (front) {
            front.set(power);
        }
        synchronized (back) {
            back.set(power);
        }
    }
    
    public double getFrontEncoderPosition() {
        return frontEncoder.getPosition();
    }

    public double getBackEncoderPosition() {
        return backEncoder.getPosition();
    }

    public double getFrontRPM() {
        return frontEncoder.getVelocity();
    }

    public double getBackRPM() {
        return backEncoder.getVelocity();
    }
}
