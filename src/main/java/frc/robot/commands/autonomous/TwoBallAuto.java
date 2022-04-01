package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        super(drivetrain, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        addCommands(
            // Pick up ball.
            new ParallelCommandGroup(
                new LowerIntake(intake),
                moveTo(new Pose2d(0, 1, Rotation2d.fromDegrees(0)))),

            // Raise the Intake
            new RaiseIntake(intake),

            // Rotate toward Hub
            rotateTo(Rotation2d.fromDegrees(10)),

            // Shoot the balls.
            new FullAutoShooterAssembly(shooter, conveyor, drivetrain, limelight)
        );
    }
}