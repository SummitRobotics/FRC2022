package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * Full auto intake mode.
 */
public class FullAutoIntake extends CommandBase {

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Lemonlight ballDetectionLimelight;

    FullAutoIntake(Drivetrain drivetrain, Intake intake, Lemonlight ballDetectionLimelight) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.ballDetectionLimelight = ballDetectionLimelight;

        addRequirements(drivetrain, intake);
    }

    @Override
    public void initialize() {

    }
}
