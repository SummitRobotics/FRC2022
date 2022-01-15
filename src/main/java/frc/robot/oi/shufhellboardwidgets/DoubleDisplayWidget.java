package frc.robot.oi.shufhellboardwidgets;

import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Wrapper for double-write-only ShufhellBoard widgets
 */
public class DoubleDisplayWidget {

    private NetworkTableEntry entry;

    public DoubleDisplayWidget(NetworkTableEntry entry){

        this.entry = entry;
        entry.forceSetDouble(0);
    }

    /**
     * @param value the value that the diplay should show
     */
    public void setValue(double value){
        entry.setDouble(value);
    }
}