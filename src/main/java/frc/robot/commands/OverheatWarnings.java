package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.StatusPriorities;

/**
 * Uses status displays to inform the driver if a shooter motor is overheating.
 *
 * @param shooter The shooter subsystem
 */
public class OverheatWarnings extends CommandBase {
    private Shooter shooter;

    public OverheatWarnings(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        // Run checks for overheating motors

        double mainTemperature = shooter.getMainTemperature();
        double followTemperature = shooter.getFollowTemperature();

        if (mainTemperature >= 10) {
            ShuffleboardDriver.statusDisplay.addStatus(
                "Shooter motor is overheating!",
                "Temperature of main motor is " + mainTemperature + " degrees Celsius.",
                Colors.RED,
                StatusPriorities.OVERHEATING_MOTOR
            );
        }

        if (followTemperature >= 10) {
            ShuffleboardDriver.statusDisplay.addStatus(
                "Shooter motor is overheating!",
                "Temperature of follow motor is " + followTemperature + " degrees Celsius.",
                Colors.RED,
                StatusPriorities.OVERHEATING_MOTOR
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
