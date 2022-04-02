package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

import java.util.List;

public class FourBallAuto extends AutoCommand {
    public FourBallAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, Conveyor conveyor, Lemonlight limelight) {
        super(drivetrain, new Pose2d(7.583, 1.741, new Rotation2d(0.045, -1.417)));

        addCommands(
            // Pick up ball one.
            new ParallelCommandGroup(
                new LowerIntake(intake),
                moveTo(new Pose2d(7.617, 0.414, new Rotation2d(0, -0.405)))),

            new ParallelCommandGroup(
                new RaiseIntake(intake),
                new TurnByEncoderAbsolute(new Rotation2d(0.664, 3.801), drivetrain)),

            // Shoot the balls.
            new FullAutoShooterAssembly(shooter, conveyor, drivetrain, limelight),

            // Drive to the 4th ball taking a path going through the third bal
            new ParallelCommandGroup(
                new LowerIntake(intake),
                moveTo(new Pose2d(1.342, 1.213, new Rotation2d(-0.843, -0.259)), List.of(new Translation2d(5.075, 1.854)))
            ),

            // Raise intake and rotate for final move
            new ParallelCommandGroup(
                new RaiseIntake(intake),
                new TurnByEncoderAbsolute(new Rotation2d(0.956, -0.124), drivetrain)),

            // Drive to shooting position
            moveTo(new Pose2d(5.919, 1.662, new Rotation2d(2.564, 2.598))),

            // Shoot the last 2 balls
            new FullAutoShooterAssembly(shooter, conveyor, drivetrain, limelight)
        );
    }
}