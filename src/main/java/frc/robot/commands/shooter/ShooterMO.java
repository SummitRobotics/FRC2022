package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Manual override for the shooter.
 */
public class ShooterMO extends CommandBase {

    Shooter shooter;

    OIAxis controlAxis;
    OIAxis.PrioritizedAxis prioritizedControlAxis;

    OIButton controlButton;
    OIButton.PrioritizedButton prioritizedControlButton;

    SimpleButton prioritizedSimpleControlButton;


    /**
     * Manual override for the shooter.
     *
     * @param shooter the shooter subsystem
     * @param controlAxis the controller axis used to control flywheel speed
     * @param controlButton the controller button used to control hood position
     */
    public ShooterMO(Shooter shooter, OIAxis controlAxis, OIButton controlButton) {
        addRequirements(shooter);

        this.shooter = shooter;
        this.controlAxis = controlAxis;
    }

    @Override
    public void initialize() {
        shooter.stop();

        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedControlButton = controlButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        prioritizedSimpleControlButton = new SimpleButton(prioritizedControlButton::get);
    }

    @Override
    public void execute() {
        shooter.setMotorPower(controlAxis.get());
        if (prioritizedSimpleControlButton.get()) {
            shooter.toggleHoodPos();
        }
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();

        prioritizedSimpleControlButton = null;
        prioritizedControlAxis.destroy();
        prioritizedControlButton.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}