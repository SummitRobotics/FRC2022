package frc.robot.oi.inputs;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.oi.Konami;
import frc.robot.utilities.Usable;

/**
 * Wrapper class for WPI's button that allows for better management
 */
public class OIButton extends Button implements Usable {

    private ArrayList<Object> users;

    public OIButton(BooleanSupplier getter) {
        super(getter);

        users = new ArrayList<>();

        whenHeld(Konami.nonRegisteredButtonPress());
    }

    public OIButton(BooleanSupplier getter, String id) {
        super(getter);

        users = new ArrayList<>();

        whenHeld(Konami.registeredButtonPress(id));
    }

    public OIButton() {
        super();

        users = new ArrayList<>();
    }

    @Override
    public void using(Object user) {
        users.add(user);
    }

    @Override
    public void release(Object user) {
        users.remove(user);
    }

    @Override
    public boolean inUse() {
        return !users.isEmpty();
    }
}
