// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.AxisPriorities;
public class ClimbAutomation extends CommandBase {
  // Initializing Subsystems
  Climb climb;
  Drivetrain drivetrain;
  
  // Creating Climber States
  public enum climbStates{
      EXTENDED,
      RETRACTED,
      IDLE,
      SPINNING,
      UNKNOWN,
      READY
    
    }
  
  // Creating different Climb states for tracking processes
    //motors
    climbStates motorLeft;
    climbStates motorRight;
    // pivot pistons
    climbStates pivotPistLeft;
    climbStates pivotPistRight;
    //clamp pistons
    climbStates clampPistLeft;
    climbStates clampPistRight;
    //tracking the system as a whole (makes it easier to check in if statements :))
    climbStates climbSystem;
  // OI
  OIButton climbButton;
  OIButton.PrioritizedButton prioritizedClimbButton;
  /** Creates a new ClimbAutomation. */
  public ClimbAutomation(Climb climb, Drivetrain drivetrain, OIButton climbButton) {
    this.climbButton = climbButton;
    this.drivetrain = drivetrain;
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.stop();
    prioritizedClimbButton =  climbButton.prioritize(AxisPriorities.DEFAULT);
    motorLeft = climbStates.IDLE;
    motorRight = climbStates.IDLE; 
    pivotPistLeft = climbStates.UNKNOWN; 
    pivotPistRight = climbStates.UNKNOWN; 
    clampPistLeft = climbStates.UNKNOWN; 
    clampPistRight = climbStates.UNKNOWN; 
    climbSystem = climbStates.READY; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (prioritizedClimbButton.get()){



    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
