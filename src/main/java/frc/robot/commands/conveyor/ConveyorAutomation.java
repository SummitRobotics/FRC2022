package frc.robot.commands.conveyor;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Command to automatically control the conveyor based on sensor measurements.
 */
public class ConveyorAutomation extends CommandBase {

    // subsystems
    private Conveyor conveyor;
    private Intake intake;
    private Shooter shooter;

    private static final int indexLidarStop = Conveyor.MAX_INDEXED_LIDAR_DISTANCE - 3;



    public static double
        BELT_SPEED = -0.5,
        INDEX_SPEED = -0.2;

    /**
     * Conveyor Automation Constructor.
     *
     * @param conveyor The Conveyor subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     */
    public ConveyorAutomation(Conveyor conveyor, Intake intake, Shooter shooter) {
        this.conveyor = conveyor;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        ConveyorState indexState = conveyor.getIndexState();
        ConveyorState beltState = conveyor.getBeltState();
        int lidarDist = conveyor.getLidarDistance();
        Shooter.States shooterState = shooter.getState();

        double beltSpeed = 0;
        double indexSpeed = 0;

        //if nothing in bent or belt has ball but index does not, run belt
        if (beltState == ConveyorState.NONE) {
            beltSpeed = BELT_SPEED;
        } else if (indexState == ConveyorState.NONE) {
            beltSpeed = BELT_SPEED;
        }

        if ((indexState != ConveyorState.NONE) && lidarDist > indexLidarStop) {
            indexSpeed = INDEX_SPEED;
        }

        if (shooterState == Shooter.States.READY_TO_FIRE) {
            indexSpeed = -1;
            beltSpeed = BELT_SPEED;
        }

        conveyor.setIndexMotorPower(indexSpeed);
        conveyor.setBeltMotorPower(beltSpeed);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}
