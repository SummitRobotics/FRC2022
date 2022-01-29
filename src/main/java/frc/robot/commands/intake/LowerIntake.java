package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Command for lowering the intake.
 */
public class LowerIntake extends InstantCommand {

    Intake intake;

    LowerIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSolenoid(true);
        intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
        intake.setState(Intake.States.DOWN);
    }
}
