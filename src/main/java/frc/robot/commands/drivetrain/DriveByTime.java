// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
        timer.start();
        drivetrain.setBothMotorPower(power);
    }

    @Override
    public void execute() {
        if (timer.get() >= targetTime) {
            done = true;
            timer.stop();
            drivetrain.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
