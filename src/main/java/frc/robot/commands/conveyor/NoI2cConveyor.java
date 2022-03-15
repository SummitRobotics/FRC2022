// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
/**
 * 
 */

public class NoI2cConveyor extends CommandBase {
    private Conveyor conveyor;
    private Shooter shooter;
    private Intake intake;
    /** Creates a new noI2cConveyor. */

    public NoI2cConveyor(Conveyor conveyor, Shooter shooter, Intake intake) {
        this.intake = intake;
        this.conveyor = conveyor;
        this.shooter = shooter;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, conveyor, intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intake.getState() == Intake.States.DOWN) {
            conveyor.setBeltMotorPower(.5);
        } else if (shooter.getState() == Shooter.States.READY_TO_FIRE) {
            conveyor.setBeltMotorPower(.5);
            conveyor.setIndexTargetPosition(10);
        } else {
            conveyor.setIndexMotorPower(0);
            conveyor.setBeltMotorPower(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
