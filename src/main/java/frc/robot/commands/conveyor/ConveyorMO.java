package frc.robot.commands.conveyor;

import frc.robot.oi.inputs.OIAxis;
import frc.robot.subsystems.Conveyor;
import frc.robot.utilities.MOCommand;

/**
 * Manual override for the conveyor.
 */
public class ConveyorMO extends MOCommand {

    Conveyor conveyor;
    OIAxis controlAxis;

    /**
     * Manual override for the conveyor.
     *
     * @param conveyor the conveyor subsystem
     * @param controlAxis the controller axis used to manually control the conveyor
     */
    public ConveyorMO(Conveyor conveyor, OIAxis controlAxis) {
        addRequirements(conveyor);
        addUsed(controlAxis);

        this.conveyor = conveyor;
        this.controlAxis = controlAxis;
    }

    @Override
    public void initialize() {
        super.initialize();
        conveyor.stop();
    }

    @Override
    public void execute() {
        conveyor.setMotorPower(controlAxis.get());
    }

    @Override
    public void end(final boolean interrupted) {
        super.end(interrupted);
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
