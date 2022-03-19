package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.subsystems.Intake;

/**
 * Command for raising the intake.
 */
public class RaiseIntake extends SequentialCommandGroup {

    /**
     * Raise the intake.
     *
     * @param intake The intake subsystem
     */
    public RaiseIntake(Intake intake) {
        addCommands(

                new InstantCommand(() -> {
                    //intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
                    intake.setIntakeSolenoid(false);
                }, intake),

                new WaitCommand(0.25),

                new InstantCommand(() -> {
                    LEDs.getInstance().removeCall("Intake Down");
                    intake.stop();
                }, intake)
        );
    }
}
