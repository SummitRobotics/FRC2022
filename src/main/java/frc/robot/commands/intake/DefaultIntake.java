package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.Intake;

/**
 * Default command for Intake.
 */
public class DefaultIntake extends CommandBase {

    private final Intake intake;

    public DefaultIntake(Intake intake) {
        addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        switch (intake.getState()) {
            case UP:
                intake.setIntakeMotorPower(0);
                return;
            case DOWN:
                intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
                break;
            case UNKNOWN:
            default:
                CommandScheduler.getInstance().schedule(new RaiseIntake(intake));
                break;
        }
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
