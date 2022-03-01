package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Intake;

/**
 * Command to automatically control the conveyor based on sensor measurements.
 */
public class ConveyorAutomation extends CommandBase {
    private Conveyor conveyor;
    private Intake intake;

    public static double
        BELT_SPEED = 0.1,
        INDEX_SPEED = 0.1;

    /**
     * Conveyor Automation Constructor.
     *
     * @param conveyor The Conveyor subsystem
     * @param intake The intake subsystem
     */
    public ConveyorAutomation(Conveyor conveyor, Intake intake) {
        this.conveyor = conveyor;
        this.intake = intake;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!conveyor.getDoesBallExist() && intake.getState() == Intake.States.UP) {
            conveyor.setBeltMotorPower(0);
            conveyor.setIndexMotorPower(0);
            return;
        }

        ConveyorState beltState = conveyor.getBeltState();
        ConveyorState indexState = conveyor.getIndexState();
        int numOfBalls = 0;

        if (indexState != ConveyorState.NONE) {
            numOfBalls++;
        }
        if (beltState != ConveyorState.NONE) {
            numOfBalls++;
        }

        if (numOfBalls == 0 && intake.getState() == Intake.States.DOWN) {
            conveyor.setBeltMotorPower(BELT_SPEED);
            conveyor.setIndexMotorPower(0);
            return;
        }
        if (numOfBalls == 1 && indexState != ConveyorState.NONE) {
            conveyor.setBeltMotorPower(BELT_SPEED);
            conveyor.setIndexMotorPower(0);
            return;
        } else if (numOfBalls == 1) {
            conveyor.setIndexMotorPower(INDEX_SPEED);
            conveyor.setBeltMotorPower(BELT_SPEED);
            return;
        }
        conveyor.setIndexMotorPower(0);
        conveyor.setBeltMotorPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interupted) {
        conveyor.stop();
    }
}
