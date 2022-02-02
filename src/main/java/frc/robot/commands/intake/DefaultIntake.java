package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Intake;

/**
 * Default command for Intake.
 */
public class DefaultIntake extends CommandBase {

    private final Intake intake;
    private final Conveyor conveyor;

    /**
     * Default command for Intake.
     *
     * @param intake the intake
     * @param conveyor the conveyor
     */
    public DefaultIntake(Intake intake, Conveyor conveyor) {
        addRequirements(intake, conveyor);
        this.intake = intake;
        this.conveyor = conveyor;
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
                if ((conveyor.getFront() == ConveyorState.BLUE 
                    || conveyor.getFront() == ConveyorState.RED)
                    && (conveyor.getBack() == ConveyorState.BLUE
                    || conveyor.getBack() == ConveyorState.RED)) {
                    CommandScheduler.getInstance().schedule(new RaiseIntake(intake));
                }
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
