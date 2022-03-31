package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;

/**
 * Waits until the conveyor state changes. For use in sequential command groups.
 */
public class WaitUntilConveyor extends CommandBase {

    private Conveyor conveyor;
    private ConveyorState startConveyorState;
    
    public WaitUntilConveyor(Conveyor conveyor) {
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        startConveyorState = conveyor.getBeltState();
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBeltState() != startConveyorState;
    }
}
