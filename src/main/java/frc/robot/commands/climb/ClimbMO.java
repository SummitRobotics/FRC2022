package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Manual override for the climber.
 */
public class ClimbMO extends CommandBase {

    // the climb subsystem
    Climb climb;

    // control axis for raising and lowering arms
    OIAxis controlAxis;
    OIAxis.PrioritizedAxis prioritizedControlAxis;

    // button for the left pivot solenoid
    OIButton leftPivotButton;
    OIButton.PrioritizedButton prioritizedLeftPivotButton;

    // button for the right pivot solenoid
    OIButton rightPivotButton;
    OIButton.PrioritizedButton prioritizedRightPivotButton;

    // button for only the left detach solenoid
    OIButton leftDetachButton;
    OIButton.PrioritizedButton prioritizedLeftDetachButton;

    // button for only the right detach solenoid
    OIButton rightDetachButton;
    OIButton.PrioritizedButton prioritizedRightDetachButton;

    // button for LeftMotorPower
    OIButton leftMotorButton;
    OIButton.PrioritizedButton prioritizedLeftMotorButton;

    // button for rightMotorPower
    OIButton rightMotorButton;
    OIButton.PrioritizedButton prioritizedRightMotorButton;

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
     */
    public ClimbMO(
        Climb climb,
        OIAxis controlAxis,
        OIButton leftMotorButton,
        OIButton rightMotorButton,
        OIButton leftPivotButton,
        OIButton rightPivotButton,
        OIButton leftDetachButton,
        OIButton rightDetachButton
    ) {
        addRequirements(climb);
        this.climb = climb;
        this.controlAxis = controlAxis;
        this.leftMotorButton = leftMotorButton;
        this.rightMotorButton = rightMotorButton;
        this.leftPivotButton = leftPivotButton;
        this.rightPivotButton = rightPivotButton;
        this.leftDetachButton = leftDetachButton;
        this.rightDetachButton = rightDetachButton;
    }

    @Override
    public void initialize() {
        prioritizedControlAxis = controlAxis.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedLeftMotorButton = leftMotorButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedRightMotorButton = rightMotorButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedLeftPivotButton = leftPivotButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedRightPivotButton = rightPivotButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedLeftDetachButton = leftDetachButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        prioritizedRightDetachButton = rightDetachButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        climb.stop();
    }

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

        climb.setPivotPosLeft(prioritizedLeftPivotButton.get());
        climb.setPivotPosRight(prioritizedRightPivotButton.get());
        climb.setLeftDetachPos(prioritizedLeftDetachButton.get());
        climb.setRightDetachPos(prioritizedRightDetachButton.get());
    }

    @Override
    public void end(final boolean interrupted) {
        climb.stop();

        prioritizedControlAxis.destroy();
        prioritizedLeftMotorButton.destroy();
        prioritizedRightMotorButton.destroy();
        prioritizedLeftPivotButton.destroy();
        prioritizedRightPivotButton.destroy();
        prioritizedLeftDetachButton.destroy();
        prioritizedRightDetachButton.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

