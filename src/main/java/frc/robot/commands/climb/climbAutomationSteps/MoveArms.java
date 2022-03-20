// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb.climbAutomationSteps;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.Functions;

public class MoveArms extends CommandBase {
    private Climb climb;
    private double target;
    private final double error = 0.2;

    /** Creates a new RaiseArms. */
    public MoveArms(Climb climb, double Target) {
        this.climb = climb;
        this.target = target;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climb.setMotorPosition(target);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Functions.isWithin(climb.getLeftEncoderValue(), target, error) && Functions.isWithin(climb.getRightEncoderValue(), target, error);
    }
}
