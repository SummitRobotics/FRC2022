package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;

/**
 * Command for lowering the intake.
 */
public class LowerIntake extends InstantCommand {

    private final Intake intake;

    public LowerIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        LEDs.getInstance().addCall("Intake Down", new LEDCall(LEDPriorities.INTAKE_DOWN, LEDRange.All).ffh(Colors.BLUE, Colors.OFF));
        intake.setIntakeSolenoid(true);
        intake.setIntakeMotorPower(Intake.INTAKE_MOTOR_SPEED);
    }
}
