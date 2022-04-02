package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.TurnByEncoderAbsolute;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoTest extends AutoCommand {
    public AutoTest(Drivetrain drivetrain, Intake intake, Shooter shooter, Conveyor conveyor, Lemonlight limelight) {
        super(drivetrain, new Pose2d(6.1, 5.26, Rotation2d.fromDegrees(0)));

        addCommands(
            // Pick up ball.
            new LowerIntake(intake),
            moveTo(new Pose2d(7.1, 5.26, Rotation2d.fromDegrees(0))),

            // Raise the Intake
            new RaiseIntake(intake),
            
            // Rotate toward Hub
            rotateTo(new Rotation2d(3.081, -1.946))
        );
    }
}