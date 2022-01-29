package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Manual override for the intake.
 */
public class IntakeMO extends CommandBase {
    
    Intake intake;

    OIAxis controlAxis;
    OIAxis.PrioritizedAxis prioritizedControlAxis;

    OIButton controlButton;
    OIButton.PrioritizedButton prioritizedControlButton;

    SimpleButton prioritizedSimpleControlButton;

    /**
     * Manual override for the intake.
     *
     * @param intake the intake subsystem
     * @param controlAxis the control axis used to control the intake motor
     * @param controlButton the control button used to control the up/down position of the intake
     */
    public IntakeMO(Intake intake, OIAxis controlAxis, OIButton controlButton) {
        addRequirements(intake);

        this.intake = intake;
        this.controlAxis = controlAxis;
        this.controlButton = controlButton;
    }

    @Override
    public void initialize() {
        intake.stop();

        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedControlButton = controlButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        prioritizedSimpleControlButton = new SimpleButton(prioritizedControlButton::get);
    }

    @Override
    public void execute() {
        intake.setIntakeMotorPower(prioritizedControlAxis.get());
        if (prioritizedSimpleControlButton.get()) {
            intake.toggleIntakeSolenoid();
        }
    }

    @Override
    public void end(final boolean interrupted) {
        intake.stop();

        prioritizedSimpleControlButton = null;
        prioritizedControlButton.destroy();
        prioritizedControlAxis.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
