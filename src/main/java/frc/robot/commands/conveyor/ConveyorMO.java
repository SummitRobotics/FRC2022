package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Manual override for the conveyor.
 */
public class ConveyorMO extends CommandBase {

    Conveyor conveyor;

    OIAxis controlAxis;
    OIAxis.PrioritizedAxis prioritizedControlAxis;

    OIButton frontMotor;
    OIButton.PrioritizedButton prioritizedFrontButton;

    OIButton backMotor;
    OIButton.PrioritizedButton prioritizedBackButton;

    /**
     * Manual override for the conveyor.
     *
     * @param conveyor the conveyor subsystem
     * @param controlAxis the controller axis used to manually control the conveyor
     * @param frontMotor the button to only move the front motor.
     * @param backMotor the button to only move the rear motor.
     */
    public ConveyorMO(
            Conveyor conveyor,
            OIAxis controlAxis,
            OIButton frontMotor,
            OIButton backMotor
    ) {
        addRequirements(conveyor);

        this.conveyor = conveyor;
        this.controlAxis = controlAxis;
        this.frontMotor = frontMotor;
        this.backMotor = backMotor;
    }

    @Override
    public void initialize() {
        conveyor.stop();

        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedBackButton = backMotor.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedFrontButton = frontMotor.prioritize(AxisPriorities.MANUAL_OVERRIDE);
    }

    @Override
    public void execute() {
        if (prioritizedFrontButton.get() && prioritizedBackButton.get()) {
            conveyor.setMotorPower(prioritizedControlAxis.get());
        } else if (prioritizedFrontButton.get()) {
            conveyor.setFrontMotorPower(prioritizedControlAxis.get());
        } else if (prioritizedBackButton.get()) {
            conveyor.setBackMotorPower(prioritizedControlAxis.get());
        } else {
            conveyor.setMotorPower(prioritizedControlAxis.get());
        }
    }

    @Override
    public void end(final boolean interrupted) {
        conveyor.stop();

        prioritizedControlAxis.destroy();
        prioritizedFrontButton.destroy();
        prioritizedBackButton.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
