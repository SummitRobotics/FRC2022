package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FollowTrajectoryThreaded;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.CustomVisionData;
import java.util.Arrays;

/**
 * Full auto intake mode.
 */
public class FullAutoIntake extends SequentialCommandGroup {
    FullAutoIntake(Drivetrain drivetrain, Intake intake, Lemonlight ballDetectionLimelight) {

        // Gets the best ball.
        CustomVisionData bestBall = CustomVisionData.calculateBestBall(
                Lemonlight.parseVisionData(ballDetectionLimelight.getCustomVisionData()));

        // Generates the poses.
        Pose2d startPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endPosition = new Pose2d(
                Units.inchesToMeters(bestBall.getXDistance()),
                Units.inchesToMeters(bestBall.getYDistance()),
                Rotation2d.fromDegrees(0)
        );

        // Generates the trajectory's
        TrajectoryConfig config = new TrajectoryConfig(5, 3);
        Trajectory trajectory = TrajectoryGenerator
                .generateTrajectory(Arrays.asList(startPosition, endPosition), config);
        drivetrain.setPose(startPosition);

        addCommands(
                new LowerIntake(intake),
                new FollowTrajectoryThreaded(drivetrain, trajectory),
                new RaiseIntake(intake)
        );
    }
}
