package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.shooter.FullAutoShooterNew;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.List;

public class FourBallAuto extends AutoCommand {
    public FourBallAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, Conveyor conveyor, Lemonlight limelight) {
        super(drivetrain, new Pose2d(7.6, 1.9, new Rotation2d(0, -1)));

        addCommands(
            // Move to ball 2
            new ParallelCommandGroup(
                moveTo(new Pose2d(7.6, 0.6, new Rotation2d(0, -1)), Direction.Forward),
                new LowerIntake(intake)
            ),
            new WaitCommand(0.2),

            // Rotate toward hub
            new RaiseIntake(intake),
            rotateTo(new Rotation2d(0.664, 3.801)),

            // Shoot the balls.
            new FullAutoShooterNew(drivetrain, shooter, conveyor, limelight),

            // Drive to the 4th ball taking a path going through the third bal
            new LowerIntake(intake),
            //moveToDirectional(new Pose2d(1.324, 1.336, new Rotation2d(-0.196, -0.073)), List.of(new Pose2d(5.411, 1.835, new Rotation2d(-2.063, 0.521))), Direction.Forward),
            moveToNondirectional(new Pose2d(1.387, 1.471, new Rotation2d(-0.697, -0.72)), List.of(new Translation2d(5.109, 1.854)), Direction.Forward),
            new WaitCommand(0.4),

            // Raise intake and rotate for final move
            new RaiseIntake(intake),

            // Drive to shooting position
            moveTo(new Pose2d(6.987, 1.820, new Rotation2d(1.237, 2.339)), Direction.Reverse),

            // Shoot the last 2 balls
            new FullAutoShooterNew(drivetrain, shooter, conveyor, limelight)
        );
    }
}