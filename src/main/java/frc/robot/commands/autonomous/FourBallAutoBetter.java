package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class FourBallAutoBetter extends AutoCommand {
    public FourBallAutoBetter(Drivetrain drivetrain, Intake intake, Shooter shooter, Conveyor conveyor, Lemonlight limelight) {
        super(drivetrain, new Pose2d(7.637, 1.867, new Rotation2d(0, -1)));

        addCommands(
            // Move to ball 2
            new LowerIntake(intake),
            moveTo(new Pose2d(7.6, 0.7, new Rotation2d(0, -1)), Direction.Forward),
            new WaitCommand(0.2),

            // Position for Shot at hub
            new RaiseIntake(intake),
            moveTo(new Pose2d(7.341, 1.987, new Rotation2d(0.665, 1.491)), Direction.Reverse),

            // Shoot the balls.
            new FullAutoShooterNew(drivetrain, shooter, conveyor, limelight),

            // Drive to the 4th ball taking a path going through the third bal
            new LowerIntake(intake),
            moveToNondirectional(new Pose2d(1.472, 1.4, new Rotation2d(-0.786, -0.529)), List.of(new Translation2d(5.072, 1.859)), Direction.Forward),
            new WaitCommand(0.2),

            // Raise intake and rotate for final move
            new RaiseIntake(intake),

            // Drive to shooting position
            moveTo(new Pose2d(4.831, 0.945, new Rotation2d(2.958, 2.654)), Direction.Reverse),

            // Shoot the last 2 balls
            new FullAutoShooterNew(drivetrain, shooter, conveyor, limelight)
        );
    }
}