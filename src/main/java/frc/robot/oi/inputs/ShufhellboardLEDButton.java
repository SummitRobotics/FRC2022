package frc.robot.oi.inputs;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ShufhellboardLEDButton extends LEDButton {

    private static final int
    ledMask = 0b00000000000000000000000000000010,
    pressMask = 0b00000000000000000000000000000001;

    private NetworkTableEntry entry;

	public ShufhellboardLEDButton(NetworkTableEntry entry) {
        this.entry = entry;

        entry.forceSetNumber(0);

		controller = new StartEndCommand(
			() -> setLed(true),
			() -> setLed(false)
		);
    }
    
    @Override
    public boolean get() {
        // if pressed is true report back and say we saw it being true by making it false again
        if (getPressed()) {
            setPressed(false);
            return true;
        } else {
            return false;
        }
    }

    //for later use
    private boolean getLed(){
        int value = entry.getNumber(0).intValue();
        // if the second bit is 1 return true
        return (value & ledMask) == 2;
    }

    private void setLed(boolean newState){
        int pressedState = entry.getNumber(0).intValue() & pressMask; // gets the pressed value so we preserve it
        if (newState) {
            entry.setNumber(pressedState | ledMask); // combines the pressed state with an active led state
        } else {
            entry.setNumber(pressedState); // pressedValue defaults to led being false
        }
    }

    private boolean getPressed(){
        int value = entry.getNumber(0).intValue() ;
        return (value & pressMask) == 1; // if the first bit is 1 return true
    }

    private void setPressed(boolean newState){
        int ledState = entry.getNumber(0).intValue() & ledMask; // gets the led value so we can perserve it
        if (newState) {
            entry.setNumber(ledState | pressMask); // combines the led state with an active pressed state
        } else{
            entry.setNumber(ledState); // ledState defaults to pressed being false
        }
    }
}