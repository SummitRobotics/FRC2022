package frc.robot.oi.drivers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Abstract class which defines a Generic Driver.
 */
public abstract class GenericDriver {
    protected int port;

    protected GenericDriver(int port) {
        this.port = port;
    }

    /**
     * Gets analog axis.
     *
     * @param axis the axis number
     * @return the axis value
     */
    protected double getRawAxis(int axis) {
        return DriverStation.getStickAxis(port, axis);
    }

    /**
     * Gets digital output.
     *
     * @param button the button number
     * @return whether the output is on or off
     */
    protected boolean getRawButton(int button) {
        return DriverStation.getStickButton(port, button);
    }

    /**
     * Gets POV for an XBox controller (In this class because it requires DriverStation).
     *
     * @return the POV
     */
    protected int getPOV() {
        return DriverStation.getStickPOV(port, 0);
    }

    /**
     * Gets a getter function for a digital output.
     *
     * @param button the button number
     * @return the getter function
     */
    protected BooleanSupplier getButtonGetter(int button) {
        return () -> getRawButton(button);
    }

    /**
     * Gets a getter function for an analog output.
     *
     * @param axis the axis number
     * @return the getter function
     */
    protected DoubleSupplier getAxisGetter(int axis) {
        return () -> getRawAxis(axis);
    }

    /**
     * Creates an OI button using a port.
     *
     * @param port the button port
     * @return the generated button
     */
    protected OIButton generateOIButton(int port) {
        return new OIButton(getButtonGetter(port));
    }

    /**
     * Creates an OI button with a given id using a port.
     *
     * @param port the button port
     * @param id   the button's id
     * @return the generated button
     */
    protected OIButton generateOIButton(int port, String id) {
        return new OIButton(getButtonGetter(port), id);
    }

    /**
     * Creates an axis using a port.
     *
     * @param port the axis port
     * @return the generated axis
     */
    protected OIAxis generateOIAxis(int port) {
        return new OIAxis(getAxisGetter(port));
    }


    /**
     * gets if the joystick is connected to the driver station.
     *
     * @return whether the joystick is connected.
     */
    public boolean isConnected() {
        return DriverStation.isJoystickConnected(this.port);
    }

    public boolean isXboxController() {
        return DriverStation.getJoystickIsXbox(this.port);
    }
}
