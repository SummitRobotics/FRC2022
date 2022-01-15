package frc.robot.oi.drivers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;

/**
 * Wrapper class for XBox controllers
 */
public class ControllerDriver extends GenericDriver {

	public enum DPadValues {
        UP(0, 45, 315),
        DOWN(180, 135, 225),
        RIGHT(90, 45, 135),
        LEFT(270, 225, 315);

        public int[] values;
        private DPadValues(int... values) {
            this.values = values;
        }

        public boolean isEqual(int value) {
            for (int element : values) {
                if (value == element) {
                    return true;
                }
            }
            return false;
        }
    }

	public OIButton
    buttonA,
    buttonB,
    buttonX,
    buttonY,
    buttonStart,
    buttonBack,
    rightBumper,
    leftBumper,

    dPadUp,
    dPadDown,
    dPadLeft,
    dPadRight;

    public OIAxis
    leftX,
    leftY,
    leftTrigger,
    rightX,
    rightY,
    rightTrigger;

	public ControllerDriver(int port) {
		super(port);

		buttonA = generateOIButton(Button.kA.value, "a");
		buttonB = generateOIButton(Button.kB.value, "b");
		buttonX = generateOIButton(Button.kX.value, "x");
		buttonY = generateOIButton(Button.kY.value, "y");
		buttonStart = generateOIButton(Button.kStart.value, "start");
		buttonBack = generateOIButton(Button.kBack.value, "back");
		rightBumper = generateOIButton(Button.kRightBumper.value, "rightBumper");
		leftBumper = generateOIButton(Button.kLeftBumper.value, "leftBumper");

		dPadUp = new OIButton(getDPadValue(DPadValues.UP), "up");
		dPadDown = new OIButton(getDPadValue(DPadValues.DOWN), "down");
		dPadLeft = new OIButton(getDPadValue(DPadValues.LEFT), "left");
        dPadRight = new OIButton(getDPadValue(DPadValues.RIGHT), "right");
        
        leftX = generateOIAxis(0);
        leftY = generateOIAxis(1);
        leftTrigger = generateOIAxis(2);
        rightTrigger = generateOIAxis(3);
        rightX = generateOIAxis(4);
        rightY = generateOIAxis(5);
	}

	private BooleanSupplier getDPadValue(DPadValues value) {
		return () -> value.isEqual(getPOV());
    }
}