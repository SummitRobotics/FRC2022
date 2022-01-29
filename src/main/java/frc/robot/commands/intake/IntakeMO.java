package frc.robot.commands.intake;

import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.MOCommand;
import frc.robot.utilities.SimpleButton;

/**
 * Manual override for the intake.
 */
public class IntakeMO extends MOCommand {
    
    Intake intake;
    OIAxis controlAxis;
    OIButton controlButton;
    SimpleButton sb;

    /**
     * Manual override for the intake.
     *
     * @param intake the intake subsystem
     * @param controlAxis the control axis used to control the intake motor
     * @param controlButton the control button used to control the up/down position of the intake
     * @param sb the SimpleButton
     */
    public IntakeMO(Intake intake, OIAxis controlAxis, OIButton controlButton, SimpleButton sb) {
        addRequirements(intake);
        addUsed(controlAxis, controlButton);
        sb = new SimpleButton(controlButton::get);

        this.intake = intake;
        this.controlAxis = controlAxis;
        this.sb = sb;
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.stop();
    }

    @Override
    public void execute() {
        intake.setIntakeMotorPower(controlAxis.get());
        if (sb.get()) {
            intake.toggleIntakeSolenoid();
        }
    }

    @Override
    public void end(final boolean interrupted) {
        super.end(interrupted);
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
