package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.TurnByEncoderAbsolute;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.shooter.FullAutoShooterAssembly;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TwoBallAuto extends AutoCommand {
    public TwoBallAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, Conveyor conveyor, Lemonlight limelight) {
        super(drivetrain, new Pose2d(6.1, 5.26, new Rotation2d(-1.147, 0.945)));

        addCommands(
            // Pick up ball.
            new ParallelCommandGroup(
                new LowerIntake(intake),
                moveTo(new Pose2d(5.2, 6, new Rotation2d(-0.776, 0.529)))
            ),

            new ParallelCommandGroup(
                // Raise the Intake
                new RaiseIntake(intake),
                // Rotate toward Hub
                new TurnByEncoderAbsolute(new Rotation2d(3.081, -1.946), drivetrain)
            ),

            // Shoot the balls.
            new FullAutoShooterAssembly(shooter, conveyor, drivetrain, limelight)
        );
    }
}