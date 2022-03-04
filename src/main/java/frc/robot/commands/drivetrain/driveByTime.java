// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * drive by time.
 */
public class driveByTime extends CommandBase {
    Drivetrain drivetrain;
    double timeToBe;
    double time;
    double power;
    boolean done;
    /** Creates a new driveByTime.
     *
     * @param drivetrain drivetrain
     * @param time time to drive
     * @param power power to drive, between -1, 1
    */

    public driveByTime(Drivetrain drivetrain, double time, double power) {
        this.drivetrain = drivetrain;
        this.power = power;
        timeToBe = time;
        done = false;
        addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time = System.currentTimeMillis() * 1000;
        drivetrain.setBothMotorPower(power);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (time - System.currentTimeMillis() * 1000 > timeToBe) {
            done = true;
            drivetrain.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}
