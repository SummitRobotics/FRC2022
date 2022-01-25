package frc.robot.oi.inputs;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.oi.Konami;
import frc.robot.utilities.Usable;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

/**
 * Wrapper class for WPI's button that allows for better management.
 */
public class OIButton extends Button implements Usable {

    private final ArrayList<Object> users;

    /**
     * Creates a button with just a BooleanSupplier.
     *
     * @param getter BooleanSupplier
     */
    public OIButton(BooleanSupplier getter) {
        super(getter);

        users = new ArrayList<>();

        whenHeld(Konami.nonRegisteredButtonPress());
    }

    /**
     * Creates a button with an ID as well.
     *
     * @param getter BooleanSupplier
     * @param id Button ID
     */
    public OIButton(BooleanSupplier getter, String id) {
        super(getter);

        users = new ArrayList<>();

        whenHeld(Konami.registeredButtonPress(id));
    }

    /**
     * Creates and OI Button.
     */
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
