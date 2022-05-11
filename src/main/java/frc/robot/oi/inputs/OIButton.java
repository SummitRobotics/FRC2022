package frc.robot.oi.inputs;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.oi.Konami;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Wrapper class for WPI's button that allows for better management.
 */
public class OIButton extends Button {

    /**
     * Creates a button with just a BooleanSupplier.
     *
     * @param getter BooleanSupplier
     */
    public OIButton(BooleanSupplier getter) {
        super(getter);
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
        whenHeld(Konami.registeredButtonPress(id));
    }

    /**
     * Creates and OI Button.
     */
    public OIButton() {
        super();
    }

    /**
     * Creates a Prioritized button.
     *
     * @param priority The priority
     * @return A prioritized button
     */
    public PrioritizedButton prioritize(int priority) {
        return new PrioritizedButton(Math.max(priority, 0));
    }

    // Variables used for Axis Prioritization
    private final Map<PrioritizedButton, Integer> uses = new HashMap<>();
    private int highestPriority = 0;

    /**
     * Inner class used for Button Prioritization.
     */
    public class PrioritizedButton {

        private final int priority;

        /**
         * Constructor for creating a Prioritized Button.
         *
         * @param priority The priority of the button.
         */
        private PrioritizedButton(int priority) {
            uses.put(this, priority);
            this.priority = priority;
            // Sets the highest priority if the new priority is higher
            // Than the current highest priority.
            if (priority > highestPriority) {
                highestPriority = priority;
            }
        }

        /**
         * Return the button value taking in priorities and will send the default value is not.
         *
         * @param defaultValue The default value to send if the command to low on the list.
         * @return The Axis value or the default value.
         */
        public boolean get(boolean defaultValue) {
            return priority < highestPriority ? defaultValue : OIButton.this.get();
        }

        /**
         * Returns the button value taking in Priorities.
         *
         * @return The Button value or false.
         */
        public boolean get() {
            return get(false);
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
         * Removes the button from being prioritized.
         * WARNING DO NOT USE THE PRIORITIZED BUTTON AFTER USING THIS COMMAND.
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
