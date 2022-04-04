package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;

/**
 * Manual override for the shooter.
 */
public class ShooterAtStart extends CommandBase {

    Shooter shooter;

    Conveyor conveyor;

    RollingAverage avg = new RollingAverage(5, false);

    private double speed = 1000;
    private double error = 60;

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

    public ShooterAtStart(Shooter shooter, Conveyor conveyor, double rpm){
        this(shooter, conveyor);
        speed = rpm;
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
        if (Functions.isWithin(avg.getAverage(), speed, error)) {
            shooter.setState(Shooter.States.READY_TO_FIRE);
            System.out.println("rpm: " + shooter.getShooterRPM());
        } else {
            shooter.setState(Shooter.States.NOT_SHOOTING);
        }
        //shooter.setMotorVolts(shooter.calculateVoltageFromPid(1700));
        shooter.setMotorTargetSpeed(speed);
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();
        shooter.setState(Shooter.States.NOT_SHOOTING);

    }

    @Override
    public boolean isFinished() {
        return !conveyor.doesBallExist();
    }
}