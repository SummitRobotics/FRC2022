// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnByPowerDraw extends CommandBase {
    Drivetrain drivetrain;
    //eates a new turnByPowerDraw
    public TurnByPowerDraw(Drivetrain drivetrain) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.drivetrain = drivetrain;
      addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      drivetrain.setMotorTargetSpeed(.5, .5);
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //if currents are roughly equal
        return false;
    }
}
