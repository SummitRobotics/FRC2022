// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.PIDValues;
import frc.robot.utilities.lists.StatusPriorities;

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
    Drivetrain drivetrain;

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

    // PID
    PIDController leftPID;
    PIDController rightPID;

    int badClimbs = 0;

    /**
     * Manual override for the climber. Many parameters!
     *
     * @param drivetrain The drivetrain subsystem
     * @param climb The climb subsystem
     * @param pivotButton The button to control the pivot
     * @param detachButton The button to control the static hooks
     * @param retractButton The button to retract the pivoting arms
     * @param extendButton The button to extend the pivoting arms
     * @param midpointButton The button to partially retract the pivoting arms
     */
    public ClimbSemiAuto(
        Climb climb,
        OIButton pivotButton,
        OIButton detachButton,
        OIButton retractButton,
        OIButton extendButton,
        OIButton midpointButton
    ) {
        
        this.drivetrain = drivetrain;
        this.climb = climb;

        this.leftPID = new PIDController(CLIMB_P, CLIMB_I, CLIMB_D);
        this.rightPID = new PIDController(CLIMB_P, CLIMB_I, CLIMB_D);

        this.pivotButton = pivotButton;
        this.detachButton = detachButton;
        this.retractButton = retractButton;
        this.extendButton = extendButton;
        this.midpointButton = midpointButton;

        // TODO - set these
        leftPID.setTolerance(1, 1);
        rightPID.setTolerance(1, 1);
        leftPID.setSetpoint(0);
        rightPID.setSetpoint(0);

        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        prioritizedPivotButton = pivotButton.prioritize(AxisPriorities.CLIMB);
        prioritizedDetachButton = detachButton.prioritize(AxisPriorities.CLIMB);
        prioritizedRetractButton = retractButton.prioritize(AxisPriorities.CLIMB);
        prioritizedExtendButton = extendButton.prioritize(AxisPriorities.CLIMB);
        prioritizedMidpointButton = midpointButton.prioritize(AxisPriorities.CLIMB);

        simplePrioritizedPivotButton = new SimpleButton(prioritizedPivotButton::get);
        simplePrioritizedDetachButton = new SimpleButton(prioritizedDetachButton::get);

        leftPID.reset();
        rightPID.reset();
        climb.stop();

        climb.setDetachPos(true);
        climb.setPivotPos(false);

        badClimbs = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (prioritizedExtendButton.get()) {
            leftPID.setSetpoint(climb.BACK_LIMIT);
            rightPID.setSetpoint(climb.BACK_LIMIT);
            climb.setLeftMotorPower(Functions.clampDouble(leftPID.calculate(climb.getLeftEncoderValue()), 0.8, -0.8));
            climb.setRightMotorPower(Functions.clampDouble(rightPID.calculate(climb.getLeftEncoderValue()), 0.8, -0.8));

        } else if (prioritizedMidpointButton.get()) {
            leftPID.setSetpoint(climb.GRAB_POINT);
            rightPID.setSetpoint(climb.GRAB_POINT);
            climb.setLeftMotorPower(Functions.clampDouble(leftPID.calculate(climb.getLeftEncoderValue()), 0.8, -0.8));
            climb.setRightMotorPower(Functions.clampDouble(rightPID.calculate(climb.getLeftEncoderValue()), 0.8, -0.8));

        } else if (prioritizedRetractButton.get()) {
            leftPID.setSetpoint(climb.FOWARD_LIMIT);
            rightPID.setSetpoint(climb.FOWARD_LIMIT);
            climb.setLeftMotorPower(Functions.clampDouble(leftPID.calculate(climb.getLeftEncoderValue()), 0.8, -0.8));
            climb.setRightMotorPower(Functions.clampDouble(rightPID.calculate(climb.getLeftEncoderValue()), 0.8, -0.8));

        } else {
            climb.setMotorPower(0);
        }

        if (simplePrioritizedPivotButton.get()) {
            climb.togglePivotPos();
        }



        if (simplePrioritizedDetachButton.get()) {
            if (climb.getLeftDetachPos() || climb.getRightDetachPos()) {
                climb.setDetachPos(false);
            } else {
                if (climb.isHooked()) {
                    climb.setDetachPos(true);
                    //System.out.println("unlatcherd");
                    ShuffleboardDriver.statusDisplay.removeStatus("bad_climb_sa");
                } else {
                    ShuffleboardDriver.statusDisplay.addStatus("bad_climb_sa", "climb is not safe to release " + badClimbs, Colors.RED, StatusPriorities.BAD_CLIMB);
                }
            }
            
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        leftPID.reset();
        rightPID.reset();
        leftPID.close();
        rightPID.close();
        climb.stop();
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
