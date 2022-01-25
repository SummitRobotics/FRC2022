package frc.robot.oi.inputs;

import frc.robot.utilities.Functions;
import frc.robot.utilities.Usable;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

/**
 * Wrapper for axes that allows for better management.
 */
public class OIAxis implements Usable {

    private static final double DEFAULT_DEADZONE = 0.05;

    protected DoubleSupplier getter;
    protected double deadzone;

    private final ArrayList<Object> users;

    public OIAxis(DoubleSupplier getter) {
        this(getter, DEFAULT_DEADZONE);
    }

    /**
     * Creates an OI axes this is used for better management.
     */
    public OIAxis(DoubleSupplier getter, double deadzone) {
        this.getter = getter;
        this.deadzone = deadzone;

        users = new ArrayList<>();
    }

    /**
     * Gets the position of the axis.
     *
     * @return the position
     */
    public double get() {
        double position = getter.getAsDouble();

        if (Functions.isWithin(position, 0, deadzone)) {
            return 0;
        }

        return (1 + deadzone) * position - Math.copySign(deadzone, position);
    }

    /**
     * Gets the deadzone.
     *
     * @return the deadzone
     */
    public double getDeadzone() {
        return deadzone;
    }

    /**
     * Sets the deadzone.
     *
     * @param deadzone the deadzone to set
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
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
