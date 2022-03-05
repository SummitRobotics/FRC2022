// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.*;

/**
 * drive by time.
 */
public class DriveByTime extends CommandBase {
    Drivetrain drivetrain;
    double timeToBe;
    double time;
    double power;
    boolean done;
    Timer timer;
    /** Creates a new driveByTime.
     *
     * @param drivetrain drivetrain
     * @param time time to drive
     * @param power power to drive, between -1, 1
    */

    public DriveByTime(Drivetrain drivetrain, double time, double power) {
        this.drivetrain = drivetrain;
        this.power = power;
        timeToBe = time;
        done = false;
        timer = new Timer();
        addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setBothMotorPower(power);
        done = false;
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("timemememermememememeem");
        drivetrain.setBothMotorPower(power);

        if (timer.hasElapsed(timeToBe)) {
            done = true;
            drivetrain.stop();
            timer.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return done;
        return done;
    }
}
