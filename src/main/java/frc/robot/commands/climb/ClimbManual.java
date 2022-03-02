// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;

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

    // button for only the left detach solenoid
    OIButton leftDetachButton;
    OIButton.PrioritizedButton prioritizedLeftDetachButton;
    SimpleButton simplePrioritizedLeftDetachButton;

    // button for only the right detach solenoid
    OIButton rightDetachButton;
    OIButton.PrioritizedButton prioritizedRightDetachButton;
    SimpleButton simplePrioritizedRightDetachButton;

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
        this.leftDetachButton = leftDetachButton;
        this.rightDetachButton = rightDetachButton;
        this.bothDetachButton = bothDetachButton;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedLeftMotorButton = leftMotorButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedRightMotorButton = rightMotorButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
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
        climb.updateClimbAverage();
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
            if (!climb.getLeftDetachPos()) {
                climb.setDetachPos(!(climb.getLeftDetachPos() && climb.getRightDetachPos()));
            } else if (climb.getLeftDetachPos() && climb.getRightDetachPos() && climb.isHooked()) {
                climb.setDetachPos(!(climb.getLeftDetachPos() && climb.getRightDetachPos()));
            }
        } else {
            if (simplePrioritizedLeftDetachButton.get()) {
                climb.toggleLeftDetachPos();
            }
            if (simplePrioritizedRightDetachButton.get()) {
                climb.toggleRightDetachPos();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.stop();
        prioritizedControlAxis.destroy();
        prioritizedLeftMotorButton.destroy();
        prioritizedRightMotorButton.destroy();
        prioritizedPivotButton.destroy();
        prioritizedLeftDetachButton.destroy();
        prioritizedRightDetachButton.destroy();
        prioritizedBothDetachButton.destroy();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
