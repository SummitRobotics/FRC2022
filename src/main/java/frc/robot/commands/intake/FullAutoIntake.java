package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FullAutoIntakeDrive;
import frc.robot.devices.BallLemonlight;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * puts intake down and drives until a ball is found.
 */
public class FullAutoIntake extends SequentialCommandGroup {

    public FullAutoIntake(Drivetrain drivetrain, Intake intake, BallLemonlight limelight, Conveyor conveyor) {
        addCommands(new LowerIntake(intake), new FullAutoIntakeDrive(drivetrain, limelight, conveyor, intake));
    }
    public FullAutoIntake(Drivetrain drivetrain, Intake intake, BallLemonlight limelight, Conveyor conveyor, boolean isThreeBall) {
        addCommands(new LowerIntake(intake), new FullAutoIntakeDrive(drivetrain, limelight, conveyor, intake, isThreeBall));
    }
    
}