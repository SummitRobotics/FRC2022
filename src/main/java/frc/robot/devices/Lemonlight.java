package frc.robot.devices;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utilities.RollingAverage;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * Device driver for the limelight.
 */
public class Lemonlight implements Sendable {
    private final NetworkTableEntry tv, tx, ty, ta, ledMode, camMode, pipeline, llpython;

    private final RollingAverage horizontalSmoothed;

    /**
     * Creates a new limelight object.
     *
     * @param tableName The table of the limelight. Default is "limelight"
     */
    public Lemonlight(String tableName) {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");

        llpython = limelight.getEntry("llpython");

        ledMode = limelight.getEntry("ledMode");
        camMode = limelight.getEntry("camMode");

        pipeline = limelight.getEntry("pipeline");

        horizontalSmoothed = new RollingAverage(5, false);
    }

    /**
     * Enum to describe the state of the LED.
     */
    public enum LEDModes {
        PIPELINE(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3);

        public final int value;

        LEDModes(int value) {
            this.value = value;
        }
    }

    /**
     * Enum to describe the state of the limelight camera.
     */
    public enum CamModes {
        VISION_PROCESSOR(0),
        DRIVER_CAMERA(1);

        public final int value;

        CamModes(int value) {
            this.value = value;
        }
    }

    // TODO - make right

    /**
     * TODO: STILL NEED TO IMPLEMENT THIS.
     *
     * @return if the limelight is at target.
     */
    public boolean atTarget() {
        return false;
    }

    /**
     * Sets the LED mode.
     *
     * @param mode the LED mode to switch to
     */
    public void setLEDMode(LEDModes mode) {
        ledMode.setDouble(mode.value);
    }

    /**
     * Sets the camera mode.
     *
     * @param mode the new mode
     */
    public void setCamMode(CamModes mode) {
        camMode.setDouble(mode.value);
    }

    /**
     * Sets the pipeline.
     *
     * @param pipe sets the pipeline to an int between 0 and 9
     */
    public void setPipeline(int pipe) {
        pipeline.setDouble(pipe);
    }

    /**
     * Checks to see if the limelight has a target.
     *
     * @return if limelight has a target
     */
    public boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    /**
     * Gets the horizontal offset from the limelight.
     *
     * @return the horizontal offset
     */
    public double getHorizontalOffset() {
        return tx.getDouble(0);
    }

    public double getSmoothedHorizontalOffset() {
        horizontalSmoothed.set(getHorizontalOffset());
        return horizontalSmoothed.getAverage();
    }

    /**
     * Gets the vertical offset from the limelight.
     *
     * @return the vertical offset
     */
    public double getVerticalOffset() {
        return ty.getDouble(0);
    }

    /**
     * Gets the percentage of coverage area from the limelight.
     *
     * @return the percentage of area
     */
    public double getAreaPercentage() {
        return ta.getDouble(0);
    }

    /**
     * gets a distance estimate of the target using the limelight and trig.
     * You need to check if the limelight has a target before running this
     *
     * @param mountAngle Mounting angle of the limelight
     * @param mountHeight Mounting height of the limelight
     * @param targetHeight Target Height.
     * @return the distance estimate or -1 if no target found
     */
    public double getLimelightDistanceEstimateIN(
            double mountAngle, double mountHeight, double targetHeight
    ) {
        if (!hasTarget()) {
            return -1;
        }
        return ((targetHeight - mountHeight)
                / Math.tan(((getVerticalOffset() + mountAngle) * (Math.PI / 180))))
                * 0.393701;
    }

    /**
     * Returns the custom vision data output by the limelight when in python mode.
     * In the limelight you output to the array labeled llpython.
     *
     * @return custom vision data as an ArrayList of Numbers
     */
    public ArrayList<Number> getCustomVisionDataNumbers() {
        // For some reason I get errors if I put {} straight into getNumberArray.
        Number[] defaultArray = {};
        return new ArrayList<>(Arrays.asList(llpython.getNumberArray(defaultArray)));
    }

    /**
     * Returns the custom vision data output by the limelight when in python mode.
     * In the limelight you output to the array labeled llpython.
     *
     * @return custom vision data as an ArrayList of Integers
     */
    public ArrayList<Double> getCustomVisionData() {
        ArrayList<Number> customVisionData = getCustomVisionDataNumbers();

        ArrayList<Double> output = new ArrayList<>();

        customVisionData.forEach(ele -> output.add(ele.doubleValue()));

        return output;
    }

    /**
     * Parses the custom vision data and wraps it into a data class.
     *
     * @param data The custom vision data.
     * @return The custom vision data wrapped in a data class.
     */
    public static ArrayList<CustomVisionData> parseVisionData(ArrayList<Double> data) {
        ArrayList<CustomVisionData> out = new ArrayList<>();

        data.forEach((value) -> {
            out.add(new CustomVisionData(value));
        });

        return out;
    }

    /**
     * Dumb method needed for telemetry.
     *
     * @return Data in a primitive double array.
     */
    private double[] getCustomVisionDataForTelemetry() {
        ArrayList<Double> data = getCustomVisionData();
        double[] out = new double[data.size()];

        for (int i = 0; i < data.size(); i++) {
            out[i] = data.get(i);
        }

        return out;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Lemonlight");
        builder.addDoubleProperty("verticalOffset", this::getVerticalOffset, null);
        builder.addDoubleProperty("horizontalOffset", this::getHorizontalOffset, null);
        builder.addDoubleArrayProperty("", this::getCustomVisionDataForTelemetry, null);
    }

    /**
     * Dataclass for Custom Vision Data.
     */
    public static class CustomVisionData {
        private double xAngle;
        private double yAngle;
        private Colors color;

        /**
         * Enum for Ball colors.
         */
        public enum Colors {
            BLUE,
            RED
        }

        //TODO STILL NEED TO IMPLEMENT THIS.
        private CustomVisionData(double data) {
            xAngle = 0.0;
            yAngle = 0.0;
            color = Colors.BLUE;
        }

        public double getXAngle() {
            return xAngle;
        }

        public double getYAngle() {
            return yAngle;
        }

        public Colors isBlue() {
            return color;
        }
    }
}
