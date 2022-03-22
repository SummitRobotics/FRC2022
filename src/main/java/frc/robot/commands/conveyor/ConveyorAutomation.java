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

    static NetworkTableEntry dumb = NetworkTableInstance.getDefault().getTable("chronic").getEntry("ball_count");


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
        dumb.forceSetDouble(-100);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        ConveyorState colorSensorState = conveyor.getColorSensorState();
        ConveyorState indexState = conveyor.getIndexState();
        boolean isBallIndexed = conveyor.isBallIndexed();
        Shooter.States shooterState = shooter.getState();

        if (shooterState == Shooter.States.READY_TO_FIRE && isBallIndexed) {
            conveyor.setIndexMotorPower(-1);
            conveyor.setBeltMotorPower(BELT_SPEED);
            return;
        }

        ConveyorState beltState = conveyor.getBeltState();

        int numOfBalls = 0;

        //System.out.println("--------------------------");

        if (indexState != ConveyorState.NONE) {
            numOfBalls++;
            //System.out.println("index add");
        }
        if (colorSensorState != ConveyorState.NONE) {
            //System.out.println("color add");
            numOfBalls++;

        }
        if (beltState != ConveyorState.NONE) {
            //System.out.println("belt add");
            numOfBalls++;
        }
        //System.out.println(numOfBalls);


        dumb.forceSetDouble(numOfBalls);

        Intake.States intakeState = intake.getState();
        boolean doesBallExist = conveyor.doesBallExist();

        if (!doesBallExist && intakeState == Intake.States.UP) {
            conveyor.setBeltMotorPower(0);
            conveyor.setIndexMotorPower(0);
        } else {
            if (numOfBalls == 0 && (doesBallExist || intakeState == Intake.States.DOWN)) {
                conveyor.setBeltMotorPower(BELT_SPEED);
                conveyor.setIndexMotorPower(0);
                return;
            }
            if (numOfBalls == 1 && indexState != ConveyorState.NONE && intakeState == Intake.States.DOWN) {
                conveyor.setBeltMotorPower(BELT_SPEED);
                conveyor.setIndexMotorPower(0);
                return;
            } else if (numOfBalls == 1 && intakeState == Intake.States.UP && indexState != ConveyorState.NONE) {
                conveyor.setBeltMotorPower(0);
                conveyor.setIndexMotorPower(0);
                return;
            } else if (numOfBalls == 1) {
                conveyor.setIndexMotorPower(INDEX_SPEED);
                conveyor.setBeltMotorPower(BELT_SPEED);
                return;
            }
            conveyor.setIndexMotorPower(0);
            conveyor.setBeltMotorPower(0);
        }
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
