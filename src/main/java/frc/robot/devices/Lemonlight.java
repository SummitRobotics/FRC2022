package frc.robot.devices;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utilities.RollingAverage;

/**
 * Device driver for the limelight
 */
public class Lemonlight {

    //TODO - make right
    public static final int X_OFFSET = 0;

    public static final double mountAngle = 19.1;

    //in cm
    public static final double mountHeight = 60.5;
    private double targetHeight = 230;

    private NetworkTable limelight;

    private NetworkTableEntry tv, tx, ty, ta, ledMode, camMode, pipeline;

    private RollingAverage horizontalSmoothed;

    public Lemonlight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");

        ledMode = limelight.getEntry("ledMode");
        camMode = limelight.getEntry("camMode");

        pipeline = limelight.getEntry("pipeline");

        horizontalSmoothed = new RollingAverage(5, false);
    }

    /**
     * Enum to describe the state of the LED
     */
    public enum LEDModes {
        PIPELINE(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3);

        public int value;
        private LEDModes(int value) {
            this.value = value;
        }
    }

    /**
     * Enum to describe the state of the limelight camera
     */
    public enum CamModes {
        VISION_PROCESSOR(0),
        DRIVER_CAMERA(1);

        public int value;
        private CamModes(int value) {
            this.value = value;
        }
    }

    //TODO - make right
    /**
     * @return if the limelight is at target
     */
    public boolean atTarget() {
        return false;
    }

    /**
     * Sets the LED mode
     * 
     * @return the new mode
     */
    public void setLEDMode(LEDModes mode) {
        ledMode.setDouble(mode.value);
    }

    /**
     * Sets the camera mode
     * 
     * @param mode the new mode
     */
    public void setCamMode(CamModes mode) {
        camMode.setDouble(mode.value);
    }

    /**
     * Sets the pipeline
     * 
     * @param pipe sets the pipeline to a int between 0 and 9
     */
    public void setPipeline(int pipe) {
        pipeline.setDouble(pipe);
    }

    /**
     * @return if limelight has a target
     */
    public boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    /**
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
     * @return the vertical offset
     */
    public double getVerticalOffset() {
        return ty.getDouble(0);
    }

    /**
     * @return the percentage of area
     */
    public double getAreaPercentage() {
        return ta.getDouble(0);
    }

    /**
     * gets a distance estimate of the target using the limelight
     * @param ReportedAngle the angle from 0 the limelight reports
     * @return the distance estmate
     */
    public double getLimelightDistanceEstimateIN(double reportedAngle){
        return ((targetHeight-mountHeight) / Math.tan(((reportedAngle + Lemonlight.mountAngle) * (Math.PI/180)))) * 0.393701;
    }
}