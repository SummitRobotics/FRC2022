// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.PIDValues;
import frc.robot.utilities.lists.Ports;



/**
 * ClimbSemiAuto.
 */

public class ClimbSemiAuto extends ClimbAutomation {
    protected static final double 
              CLIMB_P = PIDValues.CLIMB_P,
              CLIMB_I = PIDValues.CLIMB_I,
              CLIMB_D = PIDValues.CLIMB_D;
    // subsystems
    Climb climb;
    Drivetrain drivetrain;
    // control axis for raising and lowering arms

    // button for the pivot solenoid
    OIButton pivotButton;
    OIButton.PrioritizedButton prioritizedPivotButton;
    SimpleButton simplePrioritizedPivotButton;

    // button for only the left detach solenoid
    OIButton leftDetachButton;
    OIButton.PrioritizedButton prioritizedLeftDetachButton;
    SimpleButton simplePrioritizedLeftDetachButton;
    // button for only the right detach solenoid
    OIButton rightDetachButton;
    OIButton.PrioritizedButton prioritizedRightDetachButton;
    SimpleButton simplePrioritizedRightDetachButton;

    // button for LeftMotorPower
    OIButton zeroPositionButton;
    OIButton.PrioritizedButton prioritizedZeroPositionButton;

    // button for rightMotorPower
    OIButton fullMotorButton;
    OIButton.PrioritizedButton prioritizedFullMotorButton;

    // button for both detach solenoids
    OIButton bothDetachButton;
    OIButton.PrioritizedButton prioritizedBothDetachButton;
    SimpleButton simplePrioritizedBothDetachButton;

    /**
     * Manual override for the climber. Many parameters!
     *
     * @param climb the climb subsystem
     * @param zeroPositionButton Button to make the arms go to 0.
     * @param fullMotorButton Button to make the arms go to full extension.
     * @param pivotButton button for the pivot solenoid
     * @param leftDetachButton button for only the left detach solenoid
     * @param rightDetachButton button for only the right detach solenoid
     * @param bothDetachButton button for both detach solenoids
     * @param drivetrain drivetrain
     * @param mainButton main button for ClimbAutomation
     */
    public ClimbSemiAuto(
        Climb climb,
        OIButton zeroPositionButton,
        OIButton fullMotorButton,
        OIButton pivotButton,
        OIButton leftDetachButton,
        OIButton rightDetachButton,
        OIButton bothDetachButton, 
        Drivetrain drivetrain, 
        OIButton mainButton
    ) {
        
        super(climb, drivetrain, mainButton);
        this.drivetrain = drivetrain;
        addRequirements(climb);
        this.climb = climb;
        this.zeroPositionButton = zeroPositionButton;
        this.fullMotorButton = fullMotorButton;
        this.pivotButton = pivotButton;
        this.leftDetachButton = leftDetachButton;
        this.rightDetachButton = rightDetachButton;
        this.bothDetachButton = bothDetachButton;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        prioritizedZeroPositionButton = zeroPositionButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedFullMotorButton = fullMotorButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedPivotButton = pivotButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedLeftDetachButton = leftDetachButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedRightDetachButton = rightDetachButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedBothDetachButton = bothDetachButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        simplePrioritizedPivotButton = new SimpleButton(prioritizedPivotButton::get);
        simplePrioritizedLeftDetachButton = new SimpleButton(prioritizedLeftDetachButton::get);
        simplePrioritizedRightDetachButton = new SimpleButton(prioritizedRightDetachButton::get);
        simplePrioritizedBothDetachButton = new SimpleButton(prioritizedBothDetachButton::get);

        climb.stop();

        climb.setDetachPos(false);
        climb.setPivotPos(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (prioritizedZeroPositionButton.get()) {
            
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
