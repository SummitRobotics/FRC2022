package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Ports;

/**
 * Manual override for the conveyor.
 */
public class PneumaticTestCommand extends CommandBase {
    Conveyor conveyor;

    OIButton button0;
    OIButton.PrioritizedButton priButton0;
    OIButton button1;
    OIButton.PrioritizedButton priButton1;
    OIButton button2;
    OIButton.PrioritizedButton priButton2;
    OIButton button3;
    OIButton.PrioritizedButton priButton3;
    OIButton button4;
    OIButton.PrioritizedButton priButton4;
    OIButton button5;
    OIButton.PrioritizedButton priButton5;
    OIButton button6;
    OIButton.PrioritizedButton priButton6;
    OIButton button7;
    OIButton.PrioritizedButton priButton7;

    Solenoid solenoid0;
    Solenoid solenoid1;
    Solenoid solenoid2;
    Solenoid solenoid3;
    Solenoid solenoid4;
    Solenoid solenoid5;
    Solenoid solenoid6;
    Solenoid solenoid7;

    /**
     * PCM test commnand.
     *
     * @param button0 button0
     * @param button1 button1
     * @param button2 button2
     * @param button3 button3
     * @param button4 button4
     * @param button5 button5
     * @param button6 button6
     * @param button7 button7
     */
    public PneumaticTestCommand(
        OIButton button0,
        OIButton button1,
        OIButton button2,
        OIButton button3,
        OIButton button4,
        OIButton button5,
        OIButton button6,
        OIButton button7
    ) {
        this.button0 = button0;
        this.button1 = button1;
        this.button2 = button2;
        this.button3 = button3;

        this.button4 = button4;
        this.button5 = button5;
        this.button6 = button6;
        this.button7 = button7;
    }

    @Override
    public void initialize() {
        conveyor.stop();

        priButton0 = button0.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);
        priButton1 = button1.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);
        priButton2 = button2.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);
        priButton3 = button3.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);

        priButton4 = button4.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);
        priButton5 = button5.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);
        priButton6 = button6.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);
        priButton7 = button7.prioritize(AxisPriorities.MANUAL_OVERRIDE + 5);

        solenoid0 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 0);
        solenoid1 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 1);
        solenoid2 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 2);
        solenoid3 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 3);
        solenoid4 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 4);
        solenoid5 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 5);
        solenoid6 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 6);
        solenoid7 = new Solenoid(Ports.PCM_1, PneumaticsModuleType.REVPH, 7);
    }

    @Override
    public void execute() {
        solenoid0.set(priButton0.get());
        solenoid1.set(priButton1.get());
        solenoid2.set(priButton2.get());
        solenoid3.set(priButton3.get());

        solenoid4.set(priButton4.get());
        solenoid5.set(priButton5.get());
        solenoid6.set(priButton6.get());
        solenoid7.set(priButton7.get());
    }

    @Override
    public void end(final boolean interrupted) {
        conveyor.stop();

        priButton0.destroy();
        priButton1.destroy();
        priButton2.destroy();
        priButton3.destroy();

        priButton4.destroy();
        priButton5.destroy();
        priButton6.destroy();
        priButton7.destroy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
