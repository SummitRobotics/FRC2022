package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.LidarV3;

/**
 * Command to track the possible states of positions within the conveyor.
 */
public class ConveyorAutomation extends CommandBase {

    /**
     * Enum tracking what could be in the front or back of the conveyor.
     */
    public enum ConveyorState {
        NONE,
        BLUE,
        RED
    }
    
    private final ColorSensor colorSensor;
    private final LidarV3 lidar;
    private ConveyorState frontState;
    private ConveyorState backState;
    private String previousColorSensorMeasurement;
    private String colorSensorMeasurement;
    private double lidarDistance;

    private static final double
        MIN_LIDAR_DISTANCE = 0,
        MAX_LIDAR_DISTANCE = 0;

    /**
     * Command to track the possible states of positions within the conveyor.
     *
     * @param colorSensor the color sensor
     * @param lidar the lidar
     */
    public ConveyorAutomation(ColorSensor colorSensor, LidarV3 lidar) {
        this.colorSensor = colorSensor;
        this.lidar = lidar;

        frontState = ConveyorState.NONE;
        backState = ConveyorState.NONE;
        previousColorSensorMeasurement = "Unknown";
        colorSensorMeasurement = "Unknown";
        lidarDistance = -1.0;
    }

    @Override
    public void initialize() {
        lidar.startMeasuring();
    }

    @Override
    public void execute() {
        previousColorSensorMeasurement = colorSensorMeasurement;
        colorSensorMeasurement = colorSensor.getColorString();
        lidarDistance = lidar.getDistance();

        if (colorSensorMeasurement != previousColorSensorMeasurement) {
            if (MIN_LIDAR_DISTANCE <= lidarDistance && lidarDistance <= MAX_LIDAR_DISTANCE) {
                backState = frontState;

                if (colorSensor.getColorString() == "Blue") {
                    frontState = ConveyorState.BLUE;
                } else if (colorSensor.getColorString() == "Red") {
                    frontState = ConveyorState.RED;
                } else {
                    frontState = ConveyorState.NONE;
                }
            }
        }
    }

    @Override
    public void end(final boolean interrupted) {
        lidar.stopMeasuring();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public ConveyorState getFront() {
        return frontState;
    }

    public ConveyorState getBack() {
        return backState;
    }
}
