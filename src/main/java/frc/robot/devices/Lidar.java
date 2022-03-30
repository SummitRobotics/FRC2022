package frc.robot.devices;

/**
 * Interface for Lidar sensors on the robot.
 * There are both LidarV3 and V4 on the robot.
 */
public interface Lidar {
    // TODO - set LIDAR_MOUNT_ANGLE
    float LIDAR_MOUNT_ANGLE = 0;

    int getDistance();

    int getAverageDistance();

    double getLoopTimeMilliseconds();

    /**
     * Compensated the lidar distance for the lidar mount angle in inches.
     *
     * @param reportedDistance the distance reported by the lidar
     * @return the new corrected distance
     */
    default double getCompensatedLidarDistance(double reportedDistance) {
        return reportedDistance * Math.cos(Math.toDegrees(LIDAR_MOUNT_ANGLE)) * 0.393701;
    }
}
