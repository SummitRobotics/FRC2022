package frc.robot.commands.climb;

import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.MOCommand;
import frc.robot.utilities.SimpleButton;

/**
 * Manual override for the climber.
 */
public class ClimbMO extends MOCommand {

    // the climb subsystem
    Climb climb;
    // control axis for raising and lowering arms
    OIAxis controlAxis;
    // button for the pivot solenoid
    OIButton pivotButton;
    // button for both detach solenoids
    OIButton detachButton;
    // button for only the left detach solenoid
    OIButton leftDetachButton;
    // button for only the right detach solenoid
    OIButton rightDetachButton;
    // button to change joystick to only control the left arm
    OIButton leftArmButton;
    // button to change the joystick to only control the right arm
    OIButton rightArmButton;
    // SimpleButton for pivotButton
    SimpleButton simplePivotButton;
    // SimpleButton for detachButton
    SimpleButton simpleDetachButton;
    // SimpleButton for leftDetachButton
    SimpleButton simpleLeftDetachButton;
    // SimpleButton for rigthDetachButton
    SimpleButton simpleRightDetachButton;

    /**
     * Manual override for the climber. Many parameters!
     *
     * @param climb the climb subsystem
     * @param controlAxis control axis for raising and lowering arms
     * @param pivotButton button for the pivot solenoid
     * @param detachButton button for both detach solenoids
     * @param leftDetachButton button for only the left detach solenoid
     * @param rightDetachButton button for only the right detach solenoid
     * @param leftArmButton button to change joystick to only control the left arm
     * @param rightArmButton button to change the joystick to only control the right arm
     * @param simplePivotButton SimpleButton for pivotButton
     * @param simpleDetachButton SimpleButton for detachButton
     * @param simpleLeftDetachButton SimpleButton for leftDetachButton
     * @param simpleRightDetachButton SimpleButton for rigthDetachButton
     */
    public ClimbMO(
        Climb climb,
        OIAxis controlAxis,
        OIButton pivotButton,
        OIButton detachButton,
        OIButton leftDetachButton,
        OIButton rightDetachButton,
        OIButton leftArmButton,
        OIButton rightArmButton,
        SimpleButton simplePivotButton,
        SimpleButton simpleDetachButton,
        SimpleButton simpleLeftDetachButton,
        SimpleButton simpleRightDetachButton
    ) {
        addRequirements(climb);
        addUsed(
            controlAxis,
            pivotButton,
            detachButton,
            leftDetachButton,
            rightDetachButton,
            leftArmButton,
            rightArmButton
        );
        simplePivotButton = new SimpleButton(pivotButton::get);
        simpleDetachButton = new SimpleButton(detachButton::get);
        simpleLeftDetachButton = new SimpleButton(leftDetachButton::get);
        simpleRightDetachButton = new SimpleButton(rightDetachButton::get);
        
    }
}

