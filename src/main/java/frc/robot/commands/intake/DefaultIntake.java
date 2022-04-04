package frc.robot.commands.intake;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Intake;

/**
 * Default command for Intake.
 */
public class DefaultIntake extends CommandBase {

    private final Intake intake;
    private final Conveyor conveyor;
    private final AHRS gyro;
    // static NetworkTableEntry dumb =
    //     NetworkTableInstance.getDefault().getTable("chronic").getEntry("realy_dumb");

    /**
     * Default command for Intake.
     *
     * @param intake The intake
     * @param conveyor The conveyor
     * @param gyro The gyro
     */
    public DefaultIntake(Intake intake, Conveyor conveyor, AHRS gyro) {
        addRequirements(intake);
        this.intake = intake;
        this.conveyor = conveyor;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
       // dumb.forceSetDouble(1800);
    }

    @Override
    public void execute() {
        switch (intake.getState()) {
            case DOWN:
                if (conveyor.getBeltState() != ConveyorState.NONE && conveyor.getIndexState() != ConveyorState.NONE) {
                    intake.stop();
                    CommandScheduler.getInstance().schedule(new RaiseIntake(intake));
                } 
                //else {
                //     double robotVelocity = Math.sqrt(gyro.getVelocityX() * gyro.getVelocityX()
                //         + gyro.getVelocityY() * gyro.getVelocityY());
                //     intake.setIntakeSpeed(calculatePower(robotVelocity));
                // }
                //intake.setIntakeSpeed(dumb.getDouble(0));
                intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
                break;
            default:
                break;
        }
    }

    // Linear Regression
    // TODO - tune
    private double calculatePower(double velocity) {
        return 0 * velocity + 0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
