package frc.robot.oi.drivers;

import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.utilities.Functions;

/**
 * Wrapper class for basic joystick functionality
 */
public class JoystickDriver extends GenericDriver {

	public OIButton
	button5,
	button2,
	button3,
	button4,
    trigger;

	public OIAxis
	axisX,
	axisY,
    axisZ;
    
    private boolean
    assureZUp = false,
    assureZDown = false;

	public JoystickDriver(int port) {
		super(port);

		trigger = generateOIButton(1);

		button2 = generateOIButton(2);
		button3 = generateOIButton(3);
		button4 = generateOIButton(4);
		button5 = generateOIButton(5);

		axisX = generateOIAxis(0);
        axisY = generateOIAxis(1);
        // axisZ = generateOIAxis(2);
        
        axisZ = new OIAxis(getAxisGetter(2)) {
            @Override
            public double get() {
                double position  = (getter.getAsDouble() - 1) / -2;

                if (!assureZUp) {
                    if (position > .95) {
                        assureZUp = true;
                        //System.out.println("axis z is up");
                    }

                    return 0;
                }

                if (!assureZDown) {
                    if (position < .05) {
                        assureZDown = true;
                        //System.out.println("axis z is down, activated");
                    }

                    return 0;
                }

                if (Functions.isWithin(position, 0, deadzone)) {
                    return 0;
                }
        
                return (1 + deadzone) * position - Math.copySign(deadzone, position);
            }
        };
    }
    
    public void ReEnableJoysticCalibrationCheck(){
        assureZDown = false;
        assureZUp = false;
    }
}