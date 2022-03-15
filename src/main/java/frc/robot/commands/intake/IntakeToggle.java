// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Intake Toggle Command.
 */
public class IntakeToggle extends CommandBase {
    private final Intake intake;

    /** Creates a new IntakeToggle. 
     *
     * @param intake the intake
    */
    public IntakeToggle(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        if (intake.getState() == Intake.States.UP) {
            new LowerIntake(intake).schedule();
        } else {
            new RaiseIntake(intake).schedule();
        }
    }
}
