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

    public double getRed() {
        return colorSensor.getColor().red;
    }

    public double getGreen() {
        return colorSensor.getColor().green;
    }

    public double getBlue() {
        return colorSensor.getColor().blue;
    }

    public ColorMatchResult getColorMatchResult() {
        return colorMatcher.matchClosestColor(colorSensor.getColor());
    }

    public String checkColorString() {
        if (getColorMatchResult().color == blueTarget) {
            return "Blue";
        } else if (getColorMatchResult().color == redTarget) {
            return "Red";
        } else {
            return "Unknown";
        }
    }

    public double getConfidence() {
        return 1 - getColorMatchResult().confidence;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ColorSensor");
        builder.addDoubleProperty("red", this::getRed, null);
        builder.addDoubleProperty("green", this::getGreen, null);
        builder.addDoubleProperty("blue", this::getBlue, null);
    }
}
