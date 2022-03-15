// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.PIDValues;

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
    PIDController leftPID;
    PIDController rightPID;

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
     * @param cycleButton pivot button, but with safety features.
     */
    public ClimbSemiAuto(
        Drivetrain drivetrain,
        Climb climb,
        OIButton pivotButton,
        OIButton detachButton,
        OIButton retractButton,
        OIButton extendButton,
        OIButton midpointButton, 
        OIButton cycleButton
    ) {
        super(climb, drivetrain);
        isClimbGood = true;
        this.drivetrain = drivetrain;
        this.climb = climb;
        this.cycleButton = cycleButton;
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

        addRequirements(drivetrain, climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        prioritizedPivotButton = pivotButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedDetachButton = detachButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedRetractButton = retractButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedExtendButton = extendButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedMidpointButton = midpointButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedCycleButton = cycleButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        simplePrioritizedPivotButton = new SimpleButton(prioritizedPivotButton::get);
        simplePrioritizedDetachButton = new SimpleButton(prioritizedDetachButton::get);

        leftPID.reset();
        rightPID.reset();
        climb.stop();

        climb.setDetachPos(false);
        climb.setPivotPos(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isClimbGood) {
            if (prioritizedExtendButton.get()) {
                leftPID.setSetpoint(66);
                rightPID.setSetpoint(66);
                climb.setLeftMotorPower(leftPID.calculate(climb.getLeftEncoderValue()));
                climb.setRightMotorPower(rightPID.calculate(climb.getRightEncoderValue()));

            } else if (prioritizedMidpointButton.get()) {
                leftPID.setSetpoint(33);
                rightPID.setSetpoint(33);
                climb.setLeftMotorPower(leftPID.calculate(climb.getLeftEncoderValue()));
                climb.setRightMotorPower(rightPID.calculate(climb.getRightEncoderValue()));

            } else if (prioritizedRetractButton.get()) {
                leftPID.setSetpoint(0);
                rightPID.setSetpoint(0);
                climb.setLeftMotorPower(leftPID.calculate(climb.getLeftEncoderValue()));
                climb.setRightMotorPower(rightPID.calculate(climb.getRightEncoderValue()));

            } else if (prioritizedCycleButton.get()) {
                climb.setPivotPos(false);
                climbLeftPID.setSetpoint(5);
                climbRightPID.setSetpoint(5);
                climb.setLeftMotorPower(leftPID.calculate(climb.getLeftEncoderValue()));
                climb.setRightMotorPower(rightPID.calculate(climb.getRightEncoderValue()));
                if ((climb.getLeftLimit() || climb.getRightLimit()) 
                    && (climbRightPID.atSetpoint() && climbLeftPID.atSetpoint())) {
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
        leftPID.reset();
        rightPID.reset();
        leftPID.close();
        rightPID.close();
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
