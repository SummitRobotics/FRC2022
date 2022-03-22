// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb.climbAutomationSteps;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;

public class AutoAlign extends CommandBase {
    private Climb climb;
    private Drivetrain drive;
    private boolean oldLeftLimit;
    private boolean oldRightLimit;

    private Timer moveTime;


    /** Creates a new AutoAlign. */
    public AutoAlign(Climb climb, Drivetrain drive) {
        addRequirements(climb);
        this.climb = climb;
        this.drive = drive;
        moveTime = new Timer();
        oldLeftLimit = true;
        oldRightLimit = true;
       
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        moveTime.start();
        moveTime.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean align = false;

        if ((!climb.getLeftLimit()) && (!oldLeftLimit)) {
            drive.setLeftMotorPower(0.5);
            moveTime.reset();
            align = true;
        } else {
            drive.setLeftMotorPower(0);
        }

        if ((!climb.getRightLimit()) && (!oldRightLimit)) {
            drive.setRightMotorPower(0.5);
            moveTime.reset();
            align = true;
        } else {
            drive.setRightMotorPower(0);
        }

        if(!align){
            drive.setBothMotorPower(0.5);
        }

        oldRightLimit = climb.getRightLimit();
        oldLeftLimit = climb.getLeftLimit();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
        moveTime.reset();
        moveTime.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return moveTime.get() > 0.1;
    }
}
