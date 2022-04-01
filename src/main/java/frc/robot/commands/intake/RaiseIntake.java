package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

/**
 * Command for raising the intake.
 */
public class RaiseIntake extends SequentialCommandGroup {
    public RaiseIntake(Intake intake) {
        addCommands(

                new InstantCommand(() -> {
                    intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
                    intake.setIntakeSolenoid(false);
                }, intake),

                new WaitCommand(0.25),

                new InstantCommand(() -> {
                    intake.stop();
                }, intake)
        );
    }
}
