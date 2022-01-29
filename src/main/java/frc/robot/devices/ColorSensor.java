package frc.robot.devices;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Contains methods for interfacing with the REV Color Sensor V3.
 */
public class ColorSensor implements Sendable {

    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatcher;

    private final Color blueTarget;
    private final Color redTarget;

    /**
     * Creates a new ColorSensor object.
     */
    public ColorSensor() {
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();

        blueTarget = new Color(0.13989, 0.3818, 0.4787);
        redTarget = new Color(0.5510, 0.33178, 0.1176);

        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(redTarget);
    }

    /**
     * Gets the red value detected by the sensor.
     *
     * @return red the red value detected by the sensor
     */
    public double getRed() {
        return colorSensor.getColor().red;
    }

    /**
     * Gets the green value detected by the sensor.
     *
     * @return green the green value detected by the sensor
     */
    public double getGreen() {
        return colorSensor.getColor().green;
    }

    /**
     * Gets the blue value detected by the sensor.
     *
     * @return blue the blue value detected by the sensor
     */
    public double getBlue() {
        return colorSensor.getColor().blue;
    }

    /**
     * Returns the ColorMatchResult containing the sensor's data.
     * This is probably not what you want to use.
     *
     * @return colorMatch the ColorMatchResult containing the sensor's data.
     */
    public ColorMatchResult getColorMatch() {
        return colorMatcher.matchClosestColor(colorSensor.getColor());
    }

    /**
     * Returns the name of the color detected.
     *
     * @return color_name the name of the color detected
     */
    public String getColorString() {
        if (getColorMatch().color == blueTarget) {
            return "Blue";
        } else if (getColorMatch().color == redTarget) {
            return "Red";
        } else {
            return "Unknown";
        }
    }

    /**
     * Returns the level of confidence in the color selection.
     * Higher numbers mean higher confidence.
     *
     * @return confidence the level of confidence in the color selection
     */
    public double getConfidence() {
        return 1 - getColorMatch().confidence;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ColorSensor");
        builder.addDoubleProperty("red", this::getRed, null);
        builder.addDoubleProperty("green", this::getGreen, null);
        builder.addDoubleProperty("blue", this::getBlue, null);
        builder.addStringProperty("detected_color", this::getColorString, null);
        builder.addDoubleProperty("confidence", this::getConfidence, null);
    }
}
