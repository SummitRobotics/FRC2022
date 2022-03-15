package frc.robot.oi.drivers;

import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import java.util.function.BooleanSupplier;

/**
 * Wrapper class for XBox controllers.
 */
public class ControllerDriver extends GenericDriver {

    /**
     * Enum for the possible DPAD values and positions.
     */
    public enum DPadValues {
        UP(0, 45, 315),
        DOWN(180, 135, 225),
        RIGHT(90, 45, 135),
        LEFT(270, 225, 315);

        public final int[] values;

        DPadValues(int... values) {
            this.values = values;
        }

        /**
         * Checks to see if a value is equal to a DPadValue.
         *
         * @param value A value to compare
         * @return a boolean weather it is equal or not
         */
        public boolean isEqual(int value) {
            for (int element : values) {
                if (value == element) {
                    return true;
                }
            }
            return false;
        }
    }

    public OIButton buttonA,
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
            dPadRight,
            dPadAny;
            
    public OIAxis leftX, leftY, leftTrigger, rightX, rightY, rightTrigger;

    /**
     * Constructor for generating a Controller Driver.
     *
     * @param port the port of the control device
     *             this is found on the driver station.
     */
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
        dPadAny = new OIButton(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !(getPOV() == -1);
            }
        }, "any");

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