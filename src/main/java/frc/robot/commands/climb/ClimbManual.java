// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.StatusPriorities;

/**
 * Like climbMo, but with safety features.
 */

public class ClimbManual extends CommandBase {
    // the climb subsystem
    Climb climb;

    // control axis for raising and lowering arms
    OIAxis controlAxis;
    OIAxis.PrioritizedAxis prioritizedControlAxis;

    // button for the pivot solenoid
    OIButton pivotButton;
    OIButton.PrioritizedButton prioritizedPivotButton;
    SimpleButton simplePrioritizedPivotButton;

    // button for LeftMotorPower
    OIButton leftMotorButton;
    OIButton.PrioritizedButton prioritizedLeftMotorButton;

    // button for rightMotorPower
    OIButton rightMotorButton;
    OIButton.PrioritizedButton prioritizedRightMotorButton;

    // button for both detach solenoids
    OIButton bothDetachButton;
    OIButton.PrioritizedButton prioritizedBothDetachButton;
    SimpleButton simplePrioritizedBothDetachButton;

    /**
     * Manual override for the climber. Many parameters!
     *
     * @param climb the climb subsystem
     * @param controlAxis control axis for raising and lowering arms
     * @param leftMotorButton Button to make the axis control the left motor.
     * @param rightMotorButton Button to make the axis control the right motor.
     * @param pivotButton button for the pivot solenoid
     * @param leftDetachButton button for only the left detach solenoid
     * @param rightDetachButton button for only the right detach solenoid
     * @param bothDetachButton button for both detach solenoids
     */
    public ClimbManual(
        Climb climb,
        OIAxis controlAxis,
        OIButton leftMotorButton,
        OIButton rightMotorButton,
        OIButton pivotButton,
        OIButton leftDetachButton,
        OIButton rightDetachButton,
        OIButton bothDetachButton
    ) {
        addRequirements(climb);
        this.climb = climb;
        this.controlAxis = controlAxis;
        this.leftMotorButton = leftMotorButton;
        this.rightMotorButton = rightMotorButton;
        this.pivotButton = pivotButton;
        this.bothDetachButton = bothDetachButton;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDs.getInstance().addCall("Climbing", new LEDCall(LEDPriorities.CLIMBING, LEDRange.All));

        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.CLIMB);
        prioritizedLeftMotorButton = leftMotorButton.prioritize(AxisPriorities.CLIMB);
        prioritizedRightMotorButton = rightMotorButton.prioritize(AxisPriorities.CLIMB);
        prioritizedPivotButton = pivotButton.prioritize(AxisPriorities.CLIMB);
        prioritizedBothDetachButton = bothDetachButton.prioritize(AxisPriorities.CLIMB);

        simplePrioritizedPivotButton = new SimpleButton(prioritizedPivotButton::get);
        simplePrioritizedBothDetachButton = new SimpleButton(prioritizedBothDetachButton::get);

        climb.stop();

        climb.setDetachPos(false);
        climb.setPivotPos(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (prioritizedLeftMotorButton.get() && prioritizedRightMotorButton.get()) {
            climb.setMotorPower(prioritizedControlAxis.get());
        } else if (prioritizedLeftMotorButton.get()) {
            climb.setLeftMotorPower(prioritizedControlAxis.get());
        } else if (prioritizedRightMotorButton.get()) {
            climb.setRightMotorPower(prioritizedControlAxis.get());
        } else {
            climb.setMotorPower(prioritizedControlAxis.get());
        }

        if (simplePrioritizedPivotButton.get()) {
            climb.togglePivotPos();
        }

        if (simplePrioritizedBothDetachButton.get()) {
            if (climb.getLeftDetachPos() || climb.getRightDetachPos()) {
                climb.setDetachPos(false);
            } else {
                if (climb.isHooked()) {
                    climb.setDetachPos(true);
                    ShuffleboardDriver.statusDisplay.removeStatus("bad_climb");
                } else {
                    ShuffleboardDriver.statusDisplay.addStatus("bad_climb", "climb is not safe to release " + Colors.RED, StatusPriorities.BAD_CLIMB);
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LEDs.getInstance().removeCall("Climbing");
        climb.stop();
        prioritizedControlAxis.destroy();
        prioritizedLeftMotorButton.destroy();
        prioritizedRightMotorButton.destroy();
        prioritizedPivotButton.destroy();
        prioritizedBothDetachButton.destroy();
        ShuffleboardDriver.statusDisplay.removeStatus("bad_climb");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
