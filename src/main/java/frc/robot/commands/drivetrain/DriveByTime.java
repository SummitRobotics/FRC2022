package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive by time.
 */
public class DriveByTime extends CommandBase {
    Drivetrain drivetrain;
    double targetTime;
    double time;
    double power;
    boolean done;
    Timer timer;

    /** Creates a new driveByTime command.
     *
     * @param drivetrain The drivetrain subsystem
     * @param time Time to drive, in seconds
     * @param power Power to drive, between -1 and 1
    */
    public DriveByTime(Drivetrain drivetrain, double time, double power) {
        this.drivetrain = drivetrain;
        this.power = power;
        targetTime = time;
        done = false;
        timer = new Timer();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setBothMotorPower(power);
        done = false;
        timer.start();
    }

    @Override
    public void execute() {
        drivetrain.setBothMotorPower(power);

        if (timer.hasElapsed(targetTime)) {
            done = true;
            drivetrain.stop();
            timer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
