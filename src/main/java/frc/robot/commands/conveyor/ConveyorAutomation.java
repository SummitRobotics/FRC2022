package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Command to automatically control the conveyor based on sensor measurements.
 */
public class ConveyorAutomation extends CommandBase {

    // subsystems
    private Conveyor conveyor;
    private Intake intake;
    private Shooter shooter;

    public static double
        BELT_SPEED = 0.5,
        INDEX_SPEED = 0.25;

    /**
     * Conveyor Automation Constructor.
     *
     * @param conveyor The Conveyor subsystem
     * @param intake The intake subsystem
     */
    public ConveyorAutomation(Conveyor conveyor, Intake intake, Shooter shooter) {
        this.conveyor = conveyor;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        ConveyorState beltState = conveyor.getBeltState();
        ConveyorState indexState = conveyor.getIndexState();
        boolean doesBallExist = conveyor.doesBallExist();
        boolean isBallIndexed = conveyor.isBallIndexed();
        Intake.States intakeState = intake.getState();
        Shooter.States shooterState = shooter.getState();

        int numOfBalls = 0;

        if (indexState != ConveyorState.NONE) {
            numOfBalls++;
        }
        if (beltState != ConveyorState.NONE) {
            numOfBalls++;
        }

        if (!doesBallExist && intakeState == Intake.States.UP) {
            conveyor.setBeltMotorPower(0);
            conveyor.setIndexMotorPower(0);
        } else {
            if (isBallIndexed && shooterState != Shooter.States.READY_TO_FIRE) {
                conveyor.setIndexMotorPower(0);
            } else if (doesBallExist) {
                conveyor.setIndexMotorPower(INDEX_SPEED);
            }
    
            if (numOfBalls == 1) {
                conveyor.setBeltMotorPower(BELT_SPEED);
            } else {
                conveyor.setBeltMotorPower(0);
            }
        }
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
