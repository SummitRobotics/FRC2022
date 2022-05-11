package frc.robot.oi.inputs;

import frc.robot.utilities.Functions;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Wrapper for axes that allows for better management.
 */
public class OIAxis {

    private static final double DEFAULT_DEADZONE = 0.08;

    protected DoubleSupplier getter;
    protected double deadzone;

    public OIAxis(DoubleSupplier getter) {
        this(getter, DEFAULT_DEADZONE);
    }

    /**
     * Creates an OI axes this is used for better management.
     *
     * @param getter A double supplier
     * @param deadzone deadzone for the axis.
     */
    public OIAxis(DoubleSupplier getter, double deadzone) {
        this.getter = getter;
        this.deadzone = deadzone;
    }

    /**
     * Gets the position of the axis.
     * THIS DOES NOT TAKE INTO ACCOUNT PRIORITIES.
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

    public PrioritizedAxis prioritize(int priority) {
        return new PrioritizedAxis(Math.max(priority, 0));
    }


    // Variables used for Axis Prioritization
    private final Map<PrioritizedAxis, Integer> uses = new HashMap<>();
    private int highestPriority = 0;

    /**
     * Inner class used for Axis Prioritization.
     */
    public class PrioritizedAxis {

        private final int priority;

        /**
         * Constructor for creating a Prioritized Axis.
         *
         * @param priority The priority of the axis.
         */
        private PrioritizedAxis(int priority) {
            uses.put(this, priority);
            this.priority = priority;
            // Sets the highest priority if the new priority is higher
            // Than the current highest priority.
            if (priority > highestPriority) {
                highestPriority = priority;
            }
        }

        /**
         * Return the axis value taking in priorities and will send the default value is not.
         *
         * @param defaultValue The default value to send if the command to low on the list.
         * @return The Axis value or the default value.
         */
        public double get(double defaultValue) {
            return priority < highestPriority ? defaultValue : OIAxis.this.get();
        }

        /**
         * Returns the axis value taking in Priorities.
         *
         * @return The Axis value or 0.
         */
        public double get() {
            return get(0);
        }

        /**
         * Returns whether the output value is real or the default.
         *
         * @return A boolean on whether the out value is real.
         */
        public boolean isValueReal() {
            return priority >= highestPriority;
        }

        /**
         * Removes the axis from being used.
         * WARNING DO NOT USE THE PRIORITIZED AXIS AFTER USING THIS COMMAND.
         */
        public void destroy() {
            // Removes the priority from the list.
            uses.remove(this);
            
            // Only need to reassign highestPriority if the current priority
            // Is as high or higher than the highest priority.
            if (priority >= highestPriority) {
                int newHighestPriority = 0;
                for (int i : uses.values()) {
                    newHighestPriority = Math.max(i, newHighestPriority);
                }
                highestPriority = newHighestPriority;
            }
        }
    }
}
