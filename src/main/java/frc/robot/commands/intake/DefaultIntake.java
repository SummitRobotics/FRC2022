package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
            case DOWN:
                if (conveyor.getFront() != ConveyorState.NONE
                    && conveyor.getBack() != ConveyorState.NONE) {
                    intake.setIntakeMotorPower(-1.0);
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
