package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    OIButton shootButton;
    OIButton.PrioritizedButton prioritizedShootButton;

    static NetworkTableEntry dumb = NetworkTableInstance.getDefault().getTable("chronic").getEntry("realy_dumb");

    /**
     * Manual override for the shooter.
     *
     * @param shooter the shooter subsystem
     * @param controlAxis the controller axis used to control flywheel speed
     * @param controlButton the controller button used to control hood position
     * @param shootButton the button to fire.
     */
    public ShooterMO(Shooter shooter, OIAxis controlAxis, OIButton controlButton, OIButton shootButton) {
        addRequirements(shooter);

        this.shooter = shooter;
        this.controlAxis = controlAxis;
        this.controlButton = controlButton;
        this.shootButton = shootButton;
        dumb.forceSetDouble(0);
    }

    @Override
    public void initialize() {
        shooter.stop();

        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedControlButton = controlButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        if (shootButton != null) {
            prioritizedShootButton = shootButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        }

        prioritizedSimpleControlButton = new SimpleButton(prioritizedControlButton::get);
        shooter.retractHood();
    }

    @Override
    public void execute() {
        // shooter.setMotorPower(controlAxis.get());
        if (prioritizedSimpleControlButton.get()) {
            shooter.toggleHoodPos();
        }
        if (prioritizedShootButton != null && prioritizedShootButton.get(false)) {
            shooter.setState(Shooter.States.READY_TO_FIRE);
        } else {
            shooter.setState(Shooter.States.NOT_SHOOTING);
        }
        shooter.setMotorVolts(shooter.calculateVoltageFromPid(dumb.getDouble(0)));
        //shooter.setMotorTargetSpeed(dumb.getDouble(0));
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();

        prioritizedSimpleControlButton = null;
        prioritizedControlAxis.destroy();
        prioritizedControlButton.destroy();
        prioritizedShootButton.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
