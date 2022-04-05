package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.shooter.FullAutoShooterAssembly;
import frc.robot.commands.shooter.FullAutoShooterNew;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.List;

public class FourBallAutoForward extends AutoCommand {
    public FourBallAutoForward(Drivetrain drivetrain, Intake intake, Shooter shooter, Conveyor conveyor, Lemonlight limelight) {
        super(drivetrain, new Pose2d(7.583, 2.241, new Rotation2d(0, -1.417)));

        addCommands(
            // Move to ball 2
            new LowerIntake(intake),
            moveTo(new Pose2d(7.617, 0.414, new Rotation2d(0, -0.405)), Direction.Forward),

            // Rotate toward hub
            new RaiseIntake(intake),
            rotateTo(new Rotation2d(0.664, 3.801)),

            // Shoot the balls.
            new FullAutoShooterNew(drivetrain, shooter, conveyor, limelight),

            // Drive to the 4th ball taking a path going through the third bal
            new LowerIntake(intake),
            moveToNondirectional(new Pose2d(1, 1.32, new Rotation2d(-1.01, -0.85)), List.of(new Translation2d(5.1, 1.87)), Direction.Forward),
            new WaitCommand(0.4),

            // Raise intake and rotate for final move
            new RaiseIntake(intake),
            rotateTo(new Rotation2d(0.956, -0.124)),

            // Drive to shooting position
            moveTo(new Pose2d(5.919, 1.662, new Rotation2d(2.564, 2.598)), Direction.Forward),

            // Shoot the last 2 balls
            new FullAutoShooterNew(drivetrain, shooter, conveyor, limelight)
        );
    }
}