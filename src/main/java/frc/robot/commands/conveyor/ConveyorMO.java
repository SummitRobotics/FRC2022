package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Manual override for the conveyor.
 */
public class ConveyorMO extends CommandBase {

    private static final double
            BELT_RATE = 0.01,
            INDEX_RATE = 0.01;

    Conveyor conveyor;

    OIAxis controlAxis;
    OIAxis.PrioritizedAxis prioritizedControlAxis;

    OIButton beltMotor;
    OIButton.PrioritizedButton prioritizedBeltButton;

    OIButton indexMotor;
    OIButton.PrioritizedButton prioritizedIndexButton;

    // rate limiters
    private final ChangeRateLimiter beltRateLimiter;
    private final ChangeRateLimiter indexRateLimiter;

    /**
     * Manual override for the conveyor.
     *
     * @param conveyor the conveyor subsystem
     * @param controlAxis the controller axis used to manually control the conveyor
     * @param beltMotor the button to only move the belt motor.
     * @param indexMotor the button to only move the index motor.
     */
    public ConveyorMO(
            Conveyor conveyor,
            OIAxis controlAxis,
            OIButton beltMotor,
            OIButton indexMotor
    ) {
        addRequirements(conveyor);

        this.conveyor = conveyor;
        this.controlAxis = controlAxis;
        this.beltMotor = beltMotor;
        this.indexMotor = indexMotor;
        beltRateLimiter = new ChangeRateLimiter(BELT_RATE);
        indexRateLimiter = new ChangeRateLimiter(INDEX_RATE);
    }

    @Override
    public void initialize() {
        conveyor.stop();

        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedBeltButton = beltMotor.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedIndexButton = indexMotor.prioritize(AxisPriorities.MANUAL_OVERRIDE);
    }

    @Override
    public void execute() {
        if (prioritizedBeltButton.get()) {
            conveyor.setBeltMotorPower(beltRateLimiter.getRateLimitedValue(
                prioritizedControlAxis.get()));
        }

        if (prioritizedIndexButton.get()) {
            conveyor.setIndexMotorPower(indexRateLimiter.getRateLimitedValue(
                prioritizedControlAxis.get()));
        }

        if (!prioritizedIndexButton.get() && !prioritizedBeltButton.get()) {
            conveyor.setBeltMotorPower(beltRateLimiter.getRateLimitedValue(
                prioritizedControlAxis.get()));
            conveyor.setIndexMotorPower(indexRateLimiter.getRateLimitedValue(
                prioritizedControlAxis.get()));
        }
    }

    @Override
    public void end(final boolean interrupted) {
        conveyor.stop();

        prioritizedControlAxis.destroy();
        prioritizedBeltButton.destroy();
        prioritizedIndexButton.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
