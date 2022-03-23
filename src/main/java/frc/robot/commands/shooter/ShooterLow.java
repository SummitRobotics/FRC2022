package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.LEDButton;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Manual override for the shooter.
 */
public class ShooterLow extends CommandBase {

    Shooter shooter;

    OIAxis controlAxis;
    OIAxis.PrioritizedAxis prioritizedControlAxis;

    LEDButton controlButton;
    OIButton.PrioritizedButton prioritizedControlButton;

    SimpleButton prioritizedSimpleControlButton;
    private final double LOW_GOAL_SPEED = 1100;
    OIButton shootButton;
    OIButton.PrioritizedButton prioritizedShootButton;


    /**
     * Manual override for the shooter.
     *
     * @param shooter the shooter subsystem
     * @param controlAxis the controller axis used to control flywheel speed
     * @param HoodButton the controller button used to control hood position
     * @param shootButton the button to fire.
     */
    public ShooterLow(Shooter shooter, OIButton shootButton) {
        addRequirements(shooter);

        this.shooter = shooter;
        this.shootButton = shootButton;
    }

    @Override
    public void initialize() {
        shooter.stop();

        prioritizedShootButton = shootButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        shooter.retractHood();
    }

    @Override
    public void execute() {
        // shooter.setMotorPower(controlAxis.get());

        if (prioritizedShootButton.get()) {
            shooter.setState(Shooter.States.READY_TO_FIRE);
        } 
        else{
            shooter.setState(Shooter.States.NOT_SHOOTING);
        }
        shooter.setMotorTargetSpeed(LOW_GOAL_SPEED);
        //shooter.setMotorVolts(shooter.calculateVoltageFromPid(controlAxis.get() * 1500 + 1500));
        // shooter.setMotorPower(controlAxis.get());
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();
        prioritizedShootButton.destroy();
        shooter.setState(Shooter.States.NOT_SHOOTING);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
