package frc.robot.oi.inputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Class for LED Buttons.
 */
public class LEDButton extends OIButton {

    /**
     * LED interface.
     */
    public interface LED {
        void set(boolean state);
    }

    protected Command controller;

    private LED led;

    /**
     * Constructor to create a LED Button.
     *
     * @param getter a boolean supplier.
     * @param led the LED for the button.
     */
    public LEDButton(BooleanSupplier getter, LED led) {
        super(getter);
        this.led = led;

        controller = new StartEndCommand(() -> led.set(true), () -> led.set(false));
    }

    public LEDButton() {
        super();
    }

    private void triggerBind(Trigger trigger) {
        trigger.whileActiveOnce(controller);
    }

    public void setLED(boolean on){
        led.set(on);
    }

    public void toggleBind() {
        this.toggleWhenPressed(controller);
    }

    public void pressBind() {
        triggerBind(this);
    }

    public void commandBind(Command command) {
        triggerBind(new Trigger(command::isScheduled));
    }

    /**
    * binds a button light to a command and the button being pressed to a command in 1 function call
    * @param command the command for the button to be bound to
    * @param butonAction the method refrence to the action the button should activate on (eg buttonObj::whenPressed)
    * it might be """overcomplicated""" but it saves you having to explicilty define a verable
    */
	public void commandBind(Command command, Consumer<Command> butonAction){
		butonAction.accept(command);
		commandBind(command);
	}

    public void booleanSupplierBind(BooleanSupplier supplier) {
        new Trigger(supplier).whileActiveContinuous(controller);
    }
}
