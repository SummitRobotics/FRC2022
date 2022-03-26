// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.LEDPriorities;
import frc.robot.utilities.lists.PIDValues;

/**
 * ClimbSemiAuto.
 */
public class ClimbSemiAuto extends CommandBase {
    protected static final double 
              CLIMB_P = PIDValues.CLIMB_P,
              CLIMB_I = PIDValues.CLIMB_I,
              CLIMB_D = PIDValues.CLIMB_D;

    // subsystems
    Climb climb;
    private boolean isClimbGood;
    // button for the pivot solenoid
    OIButton pivotButton;
    OIButton.PrioritizedButton prioritizedPivotButton;
    SimpleButton simplePrioritizedPivotButton;

    // button for both detach solenoids
    OIButton detachButton;
    OIButton.PrioritizedButton prioritizedDetachButton;
    SimpleButton simplePrioritizedDetachButton;

    // button for fully extending the arms
    OIButton retractButton;
    OIButton.PrioritizedButton prioritizedRetractButton;

    // button for fully retracting the arms
    OIButton extendButton;
    OIButton.PrioritizedButton prioritizedExtendButton;

    // button for partially retracting the arms
    OIButton midpointButton;
    OIButton.PrioritizedButton prioritizedMidpointButton;
    //button for cycling
    OIButton cycleButton;
    OIButton.PrioritizedButton prioritizedCycleButton;
    // PID
    SparkMaxPIDController leftPID;
    SparkMaxPIDController rightPID;

    /**
     * Manual override for the climber. Many parameters!
     *
     * @param climb The climb subsystem
     * @param pivotButton The button to control the pivot
     * @param detachButton The button to control the static hooks
     * @param retractButton The button to retract the pivoting arms
     * @param extendButton The button to extend the pivoting arms
     * @param midpointButton The button to partially retract the pivoting arms
     * @param cycleButton pivot button, but with safety features.
     */
    public ClimbSemiAuto(
        Climb climb,
        OIButton pivotButton,
        OIButton detachButton,
        OIButton retractButton,
        OIButton extendButton,
        OIButton midpointButton, 
        OIButton cycleButton
    ) {
        isClimbGood = true;
        this.climb = climb;
        this.cycleButton = cycleButton;

        this.pivotButton = pivotButton;
        this.detachButton = detachButton;
        this.retractButton = retractButton;
        this.extendButton = extendButton;
        this.midpointButton = midpointButton;


        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDs.getInstance().addCall("Climbing", new LEDCall(LEDPriorities.CLIMBING, LEDRange.All));
        prioritizedPivotButton = pivotButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedDetachButton = detachButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedRetractButton = retractButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedExtendButton = extendButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedMidpointButton = midpointButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedCycleButton = cycleButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        simplePrioritizedPivotButton = new SimpleButton(prioritizedPivotButton::get);
        simplePrioritizedDetachButton = new SimpleButton(prioritizedDetachButton::get);
        climb.stop();

        climb.setDetachPos(true);
        climb.setPivotPos(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isClimbGood) {
            if (prioritizedRetractButton.get()) {
                climb.setMotorPosition(climb.FORWARD_LIMIT);


            } else if (prioritizedMidpointButton.get()) {
                climb.setMotorPosition(climb.GRAB_POINT);

            } else if (prioritizedExtendButton.get()) {
                climb.setMotorPosition(climb.BACK_LIMIT);

            } else if (prioritizedCycleButton.get()) {
                climb.setPivotPos(false);
                climb.setMotorPosition(-5);
                if ((climb.getLeftLimit() || climb.getRightLimit()) 
                    && (climb.getRightEncoderValue() >= -6 && climb.getLeftEncoderValue() >= -6)) {
                    isClimbGood = false;
                }
            } else {
                climb.setMotorPower(0);
            }
            if (simplePrioritizedPivotButton.get()) {
                climb.togglePivotPos();
            }

            if (prioritizedDetachButton.get()) {
                if (climb.isHooked()) {
                    climb.setDetachPos(false);
                }
            }
        }   
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LEDs.getInstance().removeCall("Climbing");
        prioritizedCycleButton.destroy();
        prioritizedDetachButton.destroy();
        prioritizedExtendButton.destroy();
        prioritizedMidpointButton.destroy();
        prioritizedPivotButton.destroy();
        prioritizedRetractButton.destroy();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
