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
    private final ConveyorState frontState;
    private final ConveyorState backState;

    /**
     * Command to track the possibe states of positions within the conveyor.
     *
     * @param colorSensor the color sensor
     * @param lidar the lidar
     */
    public ConveyorAutomation(ColorSensor colorSensor, LidarV3 lidar) {
        this.colorSensor = colorSensor;
        this.lidar = lidar;
        frontState = ConveyorState.NONE;
        backState = ConveyorState.NONE;

    }

    @Override
    public void initialize() {
        lidar.startMeasuring();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(final boolean interrupted) {
        lidar.stopMeasuring();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
