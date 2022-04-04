package frc.robot.devices;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Contains methods for interfacing with the REV Color Sensor V3.
 */
public class ColorSensor implements Sendable {

    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatcher;

    private final Color blueTarget;
    private final Color redTarget;
    private final Color noTarget;
    private double measuredProximity;
    private Color measuredColor;
    private double measuredLoopTime;
    private Timer loopTimer;

    private final Runnable colorSensorReader = new Runnable() {
        @Override
        public void run() {
            loopTimer.start();
            updateProximity(colorSensor.getProximity());
            updateColor(colorSensor.getColor());
            loopTimer.stop();
            updateLoopTime(loopTimer.get());
            loopTimer.reset();

        }
    };

    private final Notifier thread = new Notifier(colorSensorReader);
    private Object proximityLock = new Object();
    private Object colorLock = new Object();
    private Object loopLock = new Object();

    // The target RGB values for the specific shades of blue and red used for the balls.
    // We may need to re-adjust these later.
    private static final double
        BLUE_TARGET_RED = 0.144,
        BLUE_TARGET_GREEN = 0.383,
        BLUE_TARGET_BLUE = 0.473,
        RED_TARGET_RED = 0.565,
        RED_TARGET_GREEN = 0.319,
        RED_TARGET_BLUE = 0.116,
        NO_TARGET_RED = 0.250,
        NO_TARGET_GREEN = 0.475,
        NO_TARGET_BLUE = 0.274;

    /**
     * Creates a new ColorSensor object.
     */
    public ColorSensor() {
        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        colorMatcher = new ColorMatch();

        blueTarget = new Color(BLUE_TARGET_RED, BLUE_TARGET_GREEN, BLUE_TARGET_BLUE);
        redTarget = new Color(RED_TARGET_RED, RED_TARGET_GREEN, RED_TARGET_BLUE);
        noTarget = new Color(NO_TARGET_RED, NO_TARGET_GREEN, NO_TARGET_BLUE);

        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(redTarget);
        colorMatcher.addColorMatch(noTarget);

        measuredProximity = 0;
        measuredColor = new Color(0, 0, 0);
        measuredLoopTime = 100;
        loopTimer = new Timer();
        thread.startPeriodic(0.02);
    }

    /**
     * Returns the raw infrared value detected by the sensor.
     * If you are using this, consider putting the actual measurement in the other thread.
     *
     * @return IR the raw infrared value detected by the sensor
     */
    public double getIR() {
        return colorSensor.getIR();
    }

    /**
     * Returns the raw proximity value detected by the sensor.
     *
     * @return proximity the raw proximity value detected by the sensor
     */
    public double getProximity() {
        synchronized (proximityLock) {
            return measuredProximity;
        }
    }

    /**
     * Gets the red value detected by the sensor.
     *
     * @return red the red value detected by the sensor
     */
    public double getRed() {
        synchronized (colorLock) {
            return measuredColor.red;
        }
    }

    /**
     * Gets the green value detected by the sensor.
     *
     * @return green the green value detected by the sensor
     */
    public double getGreen() {
        synchronized (colorLock) {
            return measuredColor.green;
        }
    }

    /**
     * Gets the blue value detected by the sensor.
     *
     * @return blue the blue value detected by the sensor
     */
    public double getBlue() {
        synchronized (colorLock) {
            return measuredColor.blue;
        }
    }

    /**
     * Returns the ColorMatchResult containing the sensor's data.
     * This is probably not what you want to use.
     *
     * @return colorMatch the ColorMatchResult containing the sensor's data.
     */
    public ColorMatchResult getColorMatch() {
        synchronized (colorLock) {
            return colorMatcher.matchClosestColor(measuredColor);
        }
    }

    /**
     * Returns how long measurements took.
     *
     * @return The amount of time, in milliseconds, that it took to measure proximity and color
     */
    public double getLoopTimeMilliseconds() {
        synchronized (loopLock) {
            return measuredLoopTime * 1000;
        }
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
        } else if (getColorMatch().color == noTarget) {
            return "NoTarget";
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

    private void updateProximity(double proximity) {
        synchronized (proximityLock) {
            measuredProximity = proximity;
        }
    }

    private void updateColor(Color color) {
        synchronized (colorLock) {
            measuredColor = color;
        }
    }

    private void updateLoopTime(double time) {
        synchronized (loopLock) {
            measuredLoopTime = time;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ColorSensor");
        builder.addDoubleProperty("red", this::getRed, null);
        builder.addDoubleProperty("green", this::getGreen, null);
        builder.addDoubleProperty("blue", this::getBlue, null);
        builder.addStringProperty("detected_color", this::getColorString, null);
        builder.addDoubleProperty("confidence", this::getConfidence, null);
        builder.addDoubleProperty("infrared", this::getIR, null);
        builder.addDoubleProperty("proximity", this::getProximity, null);
    }
}
