package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.oi.inputs.OIButton;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.LEDPriorities;

/**
 * Command to test axis priorities.
 */
public class PriorityTest extends CommandBase {

    // OI
    private OIButton testButton;
    private OIButton.PrioritizedButton prioritizedButtonA;
    private OIButton.PrioritizedButton prioritizedButtonB;
    private SimpleButton simplePrioritizedButtonA;
    private SimpleButton simplePrioritizedButtonB;

    private OIButton switchButton;
    private SimpleButton simpleSwitchButton;

    public PriorityTest(OIButton testButton, OIButton switchButton) {
        this.testButton = testButton;
        this.switchButton = switchButton;
    }

    @Override
    public void initialize() {
        prioritizedButtonA = testButton.prioritize(AxisPriorities.DEFAULT);
        prioritizedButtonB = testButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        simplePrioritizedButtonA = new SimpleButton(prioritizedButtonA::get);
        simplePrioritizedButtonB = new SimpleButton(prioritizedButtonB::get);
        simpleSwitchButton = new SimpleButton(switchButton::get);
        LEDs.getInstance().addCall("TestA",
            new LEDCall(LEDPriorities.SPLINES, LEDRange.Aarms).conditionalOn(simplePrioritizedButtonA::get));
        LEDs.getInstance().addCall("TestB",
            new LEDCall(LEDPriorities.SPLINES, LEDRange.Bar).conditionalOn(simplePrioritizedButtonB::get));
    }

    @Override
    public void execute() {
        if (simpleSwitchButton.get()) {
            prioritizedButtonA.destroy();
            prioritizedButtonB.destroy();
            if (prioritizedButtonA.isValueReal()) {
                prioritizedButtonA = testButton.prioritize(AxisPriorities.DEFAULT);
                prioritizedButtonB = testButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
            } else {
                prioritizedButtonA = testButton.prioritize(AxisPriorities.MANUAL_OVERRIDE);
                prioritizedButtonB = testButton.prioritize(AxisPriorities.DEFAULT);
            }
        }

        System.out.println(prioritizedButtonA.isValueReal()
            + ", "
            + prioritizedButtonB.isValueReal());
    }

    @Override
    public void end(final boolean interrupted) {
        simpleSwitchButton = null;
        prioritizedButtonA.destroy();
        prioritizedButtonB.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}   
