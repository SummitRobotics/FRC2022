package frc.robot.utilities;

import frc.robot.devices.Lemonlight;
import java.util.ArrayList;

/**
 * Dataclass for Custom Vision Data.
 */
public class CustomVisionData {
    private final Colors color;
    private final boolean isValid;

    private final double
            xAngle,
            yAngle,
            mountYAngle,
            mountYOffset,
            mountXAngle,
            targetHeight = 5.5,
            yOffset,
            xOffset,
            distance;

    /**
     * Enum for Ball colors.
     */
    public enum Colors {
        BLUE,
        RED
    }

    //TODO NEED TO VERIFY THIS

    /**
     * Generates a new Custom Vision Data object. 6-9
     *
     * @param data The data.
     * @param mountYAngle Y Mount angle (CM)
     * @param mountYOffset Mount Y Offset (Degrees)
     * @param mountXAngle Mount X Angle (CM)
     */
    public CustomVisionData(
            int data, double mountYAngle, double mountYOffset, double mountXAngle
    ) {
        String dataString = String.valueOf(data);

        String xxAngleUnsigned = dataString.substring(2, 4) + '.' + dataString.charAt(4);
        String yyAngleUnsigned = dataString.substring(6, 8) + '.' + dataString.charAt(9);
        boolean xxPositive = dataString.charAt(1) == '0';
        boolean yyPositive = dataString.charAt(5) == '0';

        String xxAngleSigned = (xxPositive ? "" : "-") + xxAngleUnsigned;
        String yyAngleSigned = (yyPositive ? "" : "-") + yyAngleUnsigned;

        xAngle = Double.parseDouble(xxAngleSigned);
        yAngle = Double.parseDouble(yyAngleSigned);
        color = ((int) (data / 1_000_000_000)) == 1 ? Colors.RED : Colors.BLUE;
        isValid = data % 2 == 0;

        this.mountXAngle = mountXAngle;
        this.mountYAngle = mountYAngle;
        this.mountYOffset = mountYOffset;

        yOffset = calculateYOffset();
        xOffset = calculateXOffset(yOffset);
        distance = calculateDistance(xOffset, yOffset);
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

    public boolean isValid() {
        return isValid;
    }

    /**
     * Gets a distance estimate (IN).
     *
     * @return Distance estimate in inches.
     */
    private double calculateYOffset() {
        return Lemonlight.getLimelightDistanceEstimateIN(
                mountYAngle, mountYOffset, targetHeight, yAngle
        );
    }

    /**
     * Get the XOffset estimate (IN).
     *
     * @return X Offset estimate in inches.
     */
    private double calculateXOffset(double distance) {
        return Lemonlight.getXOOffsetDistanceEstimateIN(
                xAngle, mountXAngle, 0, distance
        );
    }

    private double calculateDistance(double x1, double y1) {
        return Math.sqrt((x1 * x1) + (y1 * y1));
    }

    /**
     * Lateral distance to ball (IN).
     *
     * @return Lateral distance to ball (IN).
     */
    public double getXDistance() {
        return xOffset;
    }

    /**
     * Forward and Backward distance to the ball (IN).
     *
     * @return Distance to ball (IN).
     */
    public double getYDistance() {
        return yOffset;
    }

    /**
     * Returns the total distance to the ball.
     *
     * @return The distance to the ball.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Gets the "best" ball to go for.
     *
     * @param data An array list of balls
     * @return The "best" ball
     */
    public static CustomVisionData calculateBestBall(ArrayList<CustomVisionData> data) {
        CustomVisionData best = data.get(0);
        for (CustomVisionData ele : data) {
            if (ele.isValid && best.getDistance() > ele.getDistance()) {
                best = ele;
            }
        }
        return best;
    }
}
