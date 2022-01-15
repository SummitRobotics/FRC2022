/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private static NetworkTable 
    InfoTable = NetworkTableInstance.getDefault().getTable("RobotInfo"),
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

    public static StatusDisplayWidget statusDisplay = new StatusDisplayWidget(InfoTable.getEntry("status"));

    public static NetworkTableEntry
    shooterSpeed = InfoTable.getEntry("Shooter Speed"),
    shooterTemp = InfoTable.getEntry("Shooter Temp"),
    pressure = InfoTable.getEntry("pressure"),
    shotBalls = InfoTable.getEntry("shotBalls");

    public static SendableChooser
    autoChooser = new SendableChooser<Command>();

    public static void init(){
        SmartDashboard.putData(ShufhellboardDriver.autoChooser);
    }
}
