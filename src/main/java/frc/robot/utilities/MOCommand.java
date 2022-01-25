package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * abstract class for creating MOcommands.
 */
public abstract class MOCommand extends CommandBase {

    private ArrayList<Usable> used = new ArrayList<>();

    public void addUsed(Usable... users) {
        used = new ArrayList<>(Arrays.asList(users));
    }

    @Override
    public void initialize() {
        super.initialize();

        for (Usable u : used) {
            u.using(this);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        for (Usable u : used) {
            u.release(this);
        }
    }
}
