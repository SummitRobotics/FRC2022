package frc.robot.devices.LEDs;

import java.util.HashMap;
import java.util.UUID;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.LEDs.LEDRange.Atomic;
import frc.robot.utilities.lists.Ports;


public class LEDs extends SubsystemBase {

    private static LEDs instance = null;

    private AddressableLED ledStrip;
    private AddressableLEDBuffer buffer;

    private HashMap<String, LEDCall> calls;
    private boolean callsOutOfDate;

    private int loop;

    /**
     * Gets the LED instance using the singleton pattern
     * 
     * @return
     */
    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }

        return instance;
    }

    /**
     * Creates a new LED management class, based on the robot configuration.
     */
    private LEDs() {
        ledStrip = new AddressableLED(Ports.LED_PORT);
        buffer = new AddressableLEDBuffer(Ports.LED_LENGTH);

        ledStrip.setLength(Ports.LED_LENGTH);
        ledStrip.start();

        calls = new HashMap<>();
        callsOutOfDate = true;

        loop = 0;
    }

    /**
     * Applies modifications made to the LED buffer to the LED strip
     */
    private void applyChanges() {
        ledStrip.setData(buffer);
    }

    /**
     * Adds a call to the currently active calls
     * 
     * @param name the call's name (for removal later)
     * @param call the LEDCall object
     */
    public void addCall(String name, LEDCall call) {
        callsOutOfDate = true;
        calls.putIfAbsent(name, call);
    }

    /**
     * Removes a call
     * 
     * @param name the call's name
     */
    public void removeCall(String name) {
        callsOutOfDate = true;
        calls.remove(name);
    }

     /**
     * Removes all calls
     * 
     */
    public void removeAllCalls() {
        callsOutOfDate = true;
        calls.clear();
    }
    /**
     * Reassigns the states of the LED atoms, to assure that priorities up-to date.
     * Runs whenever the currently active calls are modified
     */
    private void reassignCalls() {

        // Refreshes the LED atoms to a blank state
        for (Atomic atom : LEDRange.Atomic.values()) {
            atom.refreshCalls();
        }

        // Updates calls
        for (LEDCall call : calls.values()) {
            for (LEDRange.Atomic atom : call.getRange().getAtoms()) {
                atom.updateCall(call);
            }
        }
    }

    /**
     * Generates a unique string ID for a unnamed LEDCall
     * 
     * @return the unique ID
     */
    public String getUniqueID() {
        while (true) {
            String potentialID = UUID.randomUUID().toString();
            if (!calls.keySet().contains(potentialID)) {
                return potentialID;
            }
        }
    }

    @Override
    public void periodic() {
        loop++;

        // Reassigns calls if they have been modified
        if (callsOutOfDate) {
            reassignCalls();
            callsOutOfDate = false;
        }

        // Gets LED states
        for (LEDRange.Atomic atom : LEDRange.Atomic.values()) {
            atom.updateLEDs(buffer, loop);
        }

        applyChanges();
    }
}