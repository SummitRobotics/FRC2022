package frc.robot.commands.shooter;

import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MOCommand;
import frc.robot.utilities.SimpleButton;

/**
 * Manual override for the shooter.
 */
public class ShooterMO extends MOCommand {

    Shooter shooter;
    OIAxis controlAxis;
    OIButton controlButton;
    SimpleButton sb;

    /**
     * Manual override for the shooter.
     *
     * @param shooter the shooter subsystem
     * @param controlAxis the controller axis used to control flywheel speed
     * @param controlButton the controller button used to control 
     * @param sb the SimpleButton
     */
    public ShooterMO(Shooter shooter, OIAxis controlAxis, OIButton controlButton, SimpleButton sb) {
        addRequirements(shooter);
        addUsed(controlAxis, controlButton);
        sb = new SimpleButton(controlButton::get);

        this.shooter = shooter;
        this.controlAxis = controlAxis;
        this.sb = sb;
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.stop();
    }

    @Override
    public void execute() {
        shooter.setMotorPower(controlAxis.get());
        if (sb.get()) {
            shooter.toggleHoodPos();
        }
    }

    @Override
    public void end(final boolean interrupted) {
        super.end(interrupted);
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
