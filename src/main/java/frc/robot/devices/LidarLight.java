package frc.robot.devices;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LidarLight {
    
    public Lemonlight limelight;
    public Lidar lidar;
    
    private double workingDistance = 0;

    private int lidarBadDistanceReading = 0;
    
    public LidarLight(Lemonlight limelight, Lidar lidar) {
        this.limelight = limelight;
        this.lidar = lidar;
    }

	public double getBestDistance(){
        // double lidarDistance = lidar.getCompensatedLidarDistance(lidar.getAverageDistance());
        // double lidarDistance = lidar.getDistance();
        if (limelight.hasTarget()) {
            workingDistance = limelight.getLimelightDistanceEstimateIN(limelight.getVerticalOffset());
        }

        // SmartDashboard.putNumber("Distance from Limelight", workingDistance);
        return workingDistance;

		// if the lidar is to far from the limelight distance we use the limelight estimate beacuse it should be more reliable but less acurate
		// if (Functions.isWithin(lidarDistance, limelightDistance, acceptableLidarVSLimelightDiscrepancy)) {
        //     lidarBadDistanceReading = 0;
        //     System.out.println("distance from lidar: " + lidarDistance);
        //     return lidarDistance;
		// } else {
        //     lidarBadDistanceReading++;
        //     System.out.println("distance from limelight: " + limelightDistance);
		// 	return limelightDistance;
		// }
    }
    
    public boolean isLidarBeingBad() {
        return lidarBadDistanceReading > 20;
    }

    public double getHorizontalOffset() {
        return limelight.getHorizontalOffset();
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }
}
