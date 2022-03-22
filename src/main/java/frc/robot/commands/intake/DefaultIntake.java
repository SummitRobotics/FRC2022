package frc.robot.commands.intake;

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


    /**
     * Default command for Intake.
     *
     * @param intake The intake
     * @param conveyor The conveyor
     */
    public DefaultIntake(Intake intake, Conveyor conveyor) {
        addRequirements(intake);
        this.intake = intake;
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch (intake.getState()) {
            case DOWN:
                if (conveyor.getBeltState() != ConveyorState.NONE && conveyor.getIndexState() != ConveyorState.NONE) {
                    intake.stop();
                    CommandScheduler.getInstance().schedule(new RaiseIntake(intake));
                } else {
                    intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
                }
                break;
            default:
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
