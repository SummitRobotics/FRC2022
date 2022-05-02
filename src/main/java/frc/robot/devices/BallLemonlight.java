package frc.robot.devices;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utilities.CustomVisionData;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * Device driver for the limelight.
 */
public class BallLemonlight implements Sendable {

    public static final double
        MAIN_MOUNT_HEIGHT = 81.915,
        MAIN_MOUNT_ANGLE = 35.5,
        MAIN_TARGET_HEIGHT = 257,
        BALL_MOUNT_HEIGHT = 77.2,
        BALL_MOUNT_ANGLE = -32.0,
        BALL_MOUNT_ANGLE_X = 0.0,
        BALL_TARGET_HEIGHT = 12.065;

    private final NetworkTableEntry tv, tx, ty, ta, llpython;
    private NetworkTableEntry camMode, pipeline, ledMode;

    /**
     * gets a distance estimate IN INCHES of the target using the limelight and trig.
     * You need to check if the limelight has a target before running this
     *
     * @param mountHeight Mounting height of the limelight
     * @param mountAngle Mounting angle of the limelight
     * @param targetHeight Target Height.
     * @param targetYOffset The yOffset
     * @return the distance estimate or -1 if no target found
     */
    public static double getLimelightDistanceEstimateIN(
        double mountHeight, double mountAngle, double targetHeight, double targetYOffset
    ) {
        return ((targetHeight - mountHeight)
            / Math.tan((targetYOffset + mountAngle) * (Math.PI / 180)))
            * 0.393701;
    }

    public static double getXOOffsetDistanceEstimateIN(
        double targetAngle, double mountAngle, double mountOffset, double targetDistance
    ) {
        return targetDistance * Math.tan((targetAngle + mountAngle) * (Math.PI / 180)) * 0.393701;
    }
    /**
     * Creates a new limelight object.
     *
     * @param tableName The table of the limelight. Default is "limelight"
     */
    public BallLemonlight(String tableName) {
        NetworkTable limelight;
        limelight = NetworkTableInstance.getDefault().getTable(tableName);
        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");

        llpython = limelight.getEntry("llpython");

        //ledMode = limelight.getEntry("ledMode");
        //camMode = limelight.getEntry("camMode");
        //pipeline = limelight.getEntry("pipeline");
    }

    /**
     * Gets angles and ball colors supplied in an arraylist of double arrays.
     *
     * @return array list with double arrays, each element is for each ball
     *      element 1 of double array is for ball color; red = 0 & blue = 1
     *      element 2 of double array is horizontal offset
     *      element 3 of double array is vertical offset
     */
    public ArrayList<double[]> getCustomVisionDataReadable() {
        ArrayList<double[]> mainList = new ArrayList<double[]>();
        for (Number numbers : getCustomVisionDataNumbers()) {
            double data = numbers.doubleValue();
            double[] doubleArray = new double[3];
            try {
                if (ballExists(data)) {
                    if (isBlue(data)) {
                        doubleArray[0] = 1;
                    } else {
                        doubleArray[0] = 0;
                    }
                    doubleArray[1] = getCustomDataOffsetAngle(data, true);
                    doubleArray[2] = getCustomDataOffsetAngle(data, false);
                    mainList.add(doubleArray);
                }
            } catch (Exception e) {
                //TODO: handle exception
                System.out.println("Limelight Exception " + e);
            }
        }
        return mainList;
    }
    /** gets an angle from a number given by custom intake code.
     *
     * @param number the number to get the angle of, should be for ball tracking
     * @param isHorizontal whether or not you want it to be horizonal
     * @return the angle
     */

    public double getCustomDataOffsetAngle(double number, boolean isHorizontal) {
        String numString = new BigDecimal(number).toPlainString();
        Double angle;
        if (isHorizontal) {
            angle = Double.valueOf(numString.substring(2, 5)) / 10;
            // System.out.println("value at 1" + numString.charAt(1));
            if (numString.charAt(1) == '1') {
                angle *= -1;
            }
            System.out.println("x angle: " + angle);
        } else {
            System.out.println(numString);
            angle = Double.valueOf(numString.substring(6, 9)) / 10;
            //System.out.println("value at 6: " + numString.charAt(6));
            if (numString.charAt(5) == '1') {
                angle *= -1;

            }
            System.out.println("y angle: " + angle);
        }
        return angle;
    }

    /**
     * checks to see if ball number given by data is legitimate.
     *
     * @param number number given by custom vision code
     * @return whether or not ball exists, used to check before doing processing
     */

    public boolean ballExists(double number) {
        return number % 2 == 0;
    }
    /**
     * checks if ball is blue from number.
     *
     * @param number the number to check
     * @return if ball is blue
     */

    public boolean isBlue(double number) {
        return String.valueOf(number).startsWith("2");
    }

    /**
     * Returns the custom vision data output by the limelight when in python mode.
     * In the limelight you output to the array labeled llpython.
     *
     * @return custom vision data as an ArrayList of Numbers
     */
    public ArrayList<Number> getCustomVisionDataNumbers() {
        Number[] defaultArray = {};
        return new ArrayList<>(Arrays.asList(llpython.getNumberArray(defaultArray)));
    }

    /**
     * Returns the custom vision data output by the limelight when in python mode.
     * In the limelight you output to the array labeled llpython.
     *
     * @return custom vision data as an ArrayList of Integers
     */
    public ArrayList<Integer> getCustomVisionData() {
        ArrayList<Number> customVisionData = getCustomVisionDataNumbers();

        ArrayList<Integer> output = new ArrayList<>();

        customVisionData.forEach(ele -> output.add(ele.intValue()));

        return output;
    }

    /**
     * Parses the custom vision data and wraps it into a data class.
     *
     * @param data The custom vision data.
     * @return The custom vision data wrapped in a data class.
     */
    public static ArrayList<CustomVisionData> parseVisionData(ArrayList<Integer> data) {
        ArrayList<CustomVisionData> out = new ArrayList<>();

        data.forEach((value) -> {
            out.add(new CustomVisionData(
                value,
                BALL_MOUNT_ANGLE, BALL_MOUNT_HEIGHT, BALL_MOUNT_ANGLE_X, BALL_TARGET_HEIGHT));
        });

        return out;
    }

    /**
     * Dumb method needed for telemetry.
     *
     * @return Data in a primitive double array.
     */
    private double[] getCustomVisionDataForTelemetry() {
        ArrayList<Integer> data = getCustomVisionData();
        double[] out = new double[data.size()];

        for (int i = 0; i < data.size(); i++) {
            out[i] = data.get(i);
        }

        return out;
    }

    private double getFirstInstanceCustomVisionData() {
        return getCustomVisionDataForTelemetry()[0];
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Lemonlight");;
        builder.addDoubleArrayProperty(
            "customDataArray", this::getCustomVisionDataForTelemetry, null);
    }
}
