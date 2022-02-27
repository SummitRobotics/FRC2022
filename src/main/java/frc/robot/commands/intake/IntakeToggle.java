// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
public class IntakeToggle extends CommandBase {

  private final Intake intake;

  /** Creates a new IntakeToggle. */
  public IntakeToggle(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

    @Override
    public void initialize() {
      if (intake.getState() == Intake.States.UP){
        intake.setIntakeSolenoid(true);
        intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
        intake.setState(Intake.States.DOWN);
      }else{
        intake.setIntakeSolenoid(false);
        intake.setIntakeMotorPower(0);
        intake.setState(Intake.States.UP);
      }
    }
}
