package frc.robot.oi.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.oi.inputs.ShuffleboardLEDButton;
import frc.robot.oi.shuffleboardwidgets.StatusDisplayWidget;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Map;
import java.util.Vector;

/**
 * Driver for virtual shuffleboard buttons.
 */
public class ShuffleboardDriver {

    private static final String[] BAD_SHUFFLE_HELL_ITEMS = {"navX", "Solenoid", "Compressor", "PID", "DigitalInput"};
    private static NetworkTable InfoTable = NetworkTableInstance.getDefault().getTable("RobotInfo"),
            ButtonTable = NetworkTableInstance.getDefault().getTable("Buttons");
    public static ShuffleboardLEDButton
            //recordStart = new ShuffleboardLEDButton(ButtonTable.getEntry("record Start")),
            //intake = new ShuffleboardLEDButton(ButtonTable.getEntry("record Intake")),
            //shoot = new ShuffleboardLEDButton(ButtonTable.getEntry("record Shoot")),
            //shift = new ShuffleboardLEDButton(ButtonTable.getEntry("record Shift"));
            //finish = new ShuffleboardLEDButton(ButtonTable.getEntry("record Stop")),
            //homeTurret = new ShuffleboardLEDButton(ButtonTable.getEntry("Home Turret")),
            homeArms = new ShuffleboardLEDButton(ButtonTable.getEntry("Home Arms"));
            
        // public static DoubleDisplayWidget
        //         hoodIndicator = new DoubleDisplayWidget(InfoTable.getEntry("hood")),
        //         turretIndicator = new DoubleDisplayWidget(InfoTable.getEntry("turret"));

    public static StatusDisplayWidget statusDisplay =
            new StatusDisplayWidget(InfoTable.getEntry("status"));

    // public static NetworkTableEntry shooterSpeed = InfoTable.getEntry("Shooter Speed"),
    //         shooterTemp = InfoTable.getEntry("Shooter Temp"),
    //         pressure = InfoTable.getEntry("pressure"),
    //         shotBalls = InfoTable.getEntry("shotBalls");

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public static void init() {
        SmartDashboard.putData(ShuffleboardDriver.autoChooser);
        removeBadItems();
    }

    /**
     * uses a truly incredible amount of sin.
     * to remove the chronic pneumatic, gyro, etc. live-window
     * items from shuffleboard
     */
    public static void removeBadItems() {
        // I know this is horrifically sinfully but I at least commented it
        try {
            // gets the components map from sendableRegistry
            Field f = SendableRegistry.class.getDeclaredField("components");
            // makes components not private
            f.setAccessible(true);

            // makes components not final (from stack overflow)
            Field modifiersField = Field.class.getDeclaredField("modifiers");
            modifiersField.setAccessible(true);
            modifiersField.setInt(f, f.getModifiers() & ~Modifier.FINAL);

            // sets components to an empty list
            // f.set(null, new WeakHashMap<>());

            // goes through and removes any sendable items that are bad (by name)
            // gets the hashmap from sendableRegistry
            Map hm = (Map) f.get(null);
            // an array to hold all the objects to be removed (you can't remove objects form
            // an array while in a for each loop)
            Vector<Object> toRemove = new Vector<>();
            // loops through all the objects in the array
            for (Object x : hm.keySet()) {
                //System.out.println(x);
                // gets the name of the object
                String name = SendableRegistry.getName((Sendable) x);

                if (name != null) {

                    //System.out.println(name);
                    // loops through all bad item def
                    for (String badItem : BAD_SHUFFLE_HELL_ITEMS) {

                        // if the name of the current object contains a bad name def remove it and stop
                        // looping
                        if (name.contains(badItem)) {
                            toRemove.add(x);
                            break;
                        }
                    }
                }
                
            }
            // removes any of the objects in the toRemove array from the sendableRegistry
            toRemove.forEach((obj) -> SendableRegistry.remove((Sendable) obj));
            // toRemove.forEach((obj) -> System.out.println("remove of " +
            // SendableRegistry.getName((Sendable) obj) + " was " +
            // (SendableRegistry.remove((Sendable)obj)? "successful" : "BAD")))
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}