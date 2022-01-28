package frc.robot.oi.inputs;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * LED Button for shuffleboard.
 */
public class ShuffleboardLEDButton extends LEDButton {

    private static final int LED_MASK = 0b00000000000000000000000000000010,
            PRESS_MASK = 0b00000000000000000000000000000001;

    private final NetworkTableEntry entry;

    /**
     * Constructor.
     *
     * @param entry A network table entry.
     */
    public ShuffleboardLEDButton(NetworkTableEntry entry) {
        this.entry = entry;

        entry.forceSetNumber(0);

        controller = new StartEndCommand(() -> setLed(true), () -> setLed(false));
    }

    @Override
    public boolean get() {
        // if pressed is true report back and say we saw it being true by making it
        // false again
        if (getPressed()) {
            setPressed(false);
            return true;
        } else {
            return false;
        }
    }

    /*
     * for later use
     * private boolean getLed(){
     * int value = entry.getNumber(0).intValue();
     * // if the second bit is 1 return true
     * return (value & ledMask) == 2;
     * }
     */

    private void setLed(boolean newState) {
        int pressedState =
                // gets the pressed value so we preserve it
                entry.getNumber(0).intValue() & PRESS_MASK;
        if (newState) {
            entry.setNumber(
                    // combines the pressed state with an active led state
                    pressedState | LED_MASK);
        } else {
            // pressedValue defaults to led being false
            entry.setNumber(pressedState);
        }
    }

    private boolean getPressed() {
        int value = entry.getNumber(0).intValue();
        return (value & PRESS_MASK) == 1; // if the first bit is 1 return true
    }

    private void setPressed(boolean newState) {
        int ledState =
                // gets the LED value so we can preserve it
                entry.getNumber(0).intValue() & LED_MASK;
        if (newState) {
            // combines the LED state with an active pressed state
            entry.setNumber(ledState | PRESS_MASK);
        } else {
            // ledState defaults to pressed being false
            entry.setNumber(ledState);
        }
    }
}
