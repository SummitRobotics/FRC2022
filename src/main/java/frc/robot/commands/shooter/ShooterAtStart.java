package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Manual override for the shooter.
 */
public class ShooterAtStart extends CommandBase {

    Shooter shooter;

    Conveyor conveyor;

    RollingAverage avg = new RollingAverage(5, false);

    /**
     * Manual override for the shooter.
     *
     * @param shooter the shooter subsystem
     * @param conveyor the conveyor subsystem
     */
    public ShooterAtStart(Shooter shooter, Conveyor conveyor) {
        addRequirements(shooter);

        this.shooter = shooter;
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        shooter.stop();

        shooter.extendHood();
    }

    @Override
    public void execute() {
        // shooter.setMotorPower(controlAxis.get());
        shooter.setHoodPos(false);
        avg.update(shooter.getShooterRPM());
        if (Functions.isWithin(avg.getAverage(), 1700, 20)) {
            shooter.setState(Shooter.States.READY_TO_FIRE);
        } else {
            shooter.setState(Shooter.States.NOT_SHOOTING);
        }
        shooter.setMotorVolts(shooter.calculateVoltageFromPid(1700));
        System.out.println("feawuybiouabivbgrjfhfjfhjfhjfjfjfjfjfjffjfj");
        //shooter.setMotorTargetSpeed(dumb.getDouble(0));
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        // System.out.println("existssssss: "+ conveyor.doesBallExist());
        return /*!conveyor.doesBallExist();*/ true;
    }
}
