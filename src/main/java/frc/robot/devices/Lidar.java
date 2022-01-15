package frc.robot.devices;

public interface Lidar {
    public static final float LIDAR_MOUNT_ANGLE = 25;

	public int getDistance();
    public int getAverageDistance();
    
    /**
     * Compensated the lidar distance for the lidar mount angle in inches
     * @param reportedDistance the distance reported by the lidar
     * @return the new corrected distance
     */
    public default double getCompensatedLidarDistance(double reportedDistance){
        return reportedDistance * Math.cos(Math.toDegrees(LIDAR_MOUNT_ANGLE)) * 0.393701;
    }
}