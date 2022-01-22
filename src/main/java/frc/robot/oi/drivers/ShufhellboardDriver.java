/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi.drivers;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Map;
import java.util.Vector;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.oi.inputs.ShufhellboardLEDButton;
import frc.robot.oi.shufhellboardwidgets.DoubleDisplayWidget;
import frc.robot.oi.shufhellboardwidgets.StatusDisplayWidget;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Driver for virtual shuffleboard buttons
 */
public class ShufhellboardDriver {

    private static String[] badShuffHellItems = {"navX", "Solenoid", "Compressor"};

    private static NetworkTable InfoTable = NetworkTableInstance.getDefault().getTable("RobotInfo"),
            ButtonTable = NetworkTableInstance.getDefault().getTable("Buttons");
    public static ShufhellboardLEDButton
            recordStart = new ShufhellboardLEDButton(ButtonTable.getEntry("record Start")),
            intake = new ShufhellboardLEDButton(ButtonTable.getEntry("record Intake")),
            shoot = new ShufhellboardLEDButton(ButtonTable.getEntry("record Shoot")),
            shift = new ShufhellboardLEDButton(ButtonTable.getEntry("record Shift")),
            finish = new ShufhellboardLEDButton(ButtonTable.getEntry("record Stop")),
            homeTurret = new ShufhellboardLEDButton(ButtonTable.getEntry("Home Turret")),
            homeHood = new ShufhellboardLEDButton(ButtonTable.getEntry("Home Hood"));
    public static DoubleDisplayWidget
            hoodIndicator = new DoubleDisplayWidget(InfoTable.getEntry("hood")),
            turretIndicator = new DoubleDisplayWidget(InfoTable.getEntry("turret"));

    public static StatusDisplayWidget statusDisplay =
            new StatusDisplayWidget(InfoTable.getEntry("status"));

    public static NetworkTableEntry shooterSpeed = InfoTable.getEntry("Shooter Speed"),
            shooterTemp = InfoTable.getEntry("Shooter Temp"),
            pressure = InfoTable.getEntry("pressure"),
            shotBalls = InfoTable.getEntry("shotBalls");

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public static void init() {
        SmartDashboard.putData(ShufhellboardDriver.autoChooser);
        removeBadItems();
    }

    /**
     * uses a truly incredible amount of sin to remove the chronic pneumatic, gyro, etc livewindow
     * items from shufhellboard
     */
    public static void removeBadItems() {
        // i know this is horificaly sinfull but i at least commented it
        try {
            // gets the components map from sendableRegistery
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
            // gets the hashmap from sendableRegistery
            Map hm = (Map) f.get(null);
            // an array to hold all the bjects to be removed (you cant remove objects form
            // an array while in a for each loop)
            Vector<Object> toRemove = new Vector<>();
            // loops through all the objects in the array
            for (Object x : hm.keySet()) {
                // gets the name of the object
                String name = SendableRegistry.getName((Sendable) x);

                // loops through all bad item defs
                for (String badItem : badShuffHellItems) {

                    // if the name of the curent object contains a bad name def remove it and stop
                    // looping
                    if (name.contains(badItem)) {
                        toRemove.add(x);
                        break;
                    }
                }
            }
            // removes any of the objects in the toRemove array from the sendableRegistery
            toRemove.forEach((obj) -> SendableRegistry.remove((Sendable) obj));
            // toRemove.forEach((obj) -> System.out.println("remove of " +
            // SendableRegistry.getName((Sendable) obj) + " was " +
            // (SendableRegistry.remove((Sendable)obj)? "sucessfull" : "BAD")));
        }
        // i hope this never happens
        catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
