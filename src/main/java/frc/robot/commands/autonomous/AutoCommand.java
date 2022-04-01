package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FollowTrajectoryThreaded;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.subsystems.Drivetrain;
import java.util.List;

/**
 * Class for AutoCommands.
 */
public abstract class AutoCommand extends SequentialCommandGroup {
    protected final Drivetrain drivetrain;
    protected final TrajectoryConfig config;

    AutoCommand(Drivetrain drivetrain, Pose2d startingPosition) {

        super(new InstantCommand(() -> {
            drivetrain.setPose(startingPosition);
            drivetrain.highGear();
        }));

        this.drivetrain = drivetrain;
        this.config = drivetrain.getTrajectoryConfigHighGear();
    }

    protected Command moveTo(Pose2d end) {
        return moveTo(end, List.of());
    }

    protected Command moveTo(Pose2d end, List<Translation2d> midpoints) {
        return moveTo(drivetrain.getPose(), end, midpoints);
    }

    private Command moveTo(Pose2d start, Pose2d end, List<Translation2d> midpoints) {
        return new FollowTrajectoryThreaded(drivetrain,
                TrajectoryGenerator.generateTrajectory(start, midpoints, end, config));
    }

    // Need to make sure this works.
    protected Command rotateTo(Rotation2d angle) {
        double targetAngle = -angle.getDegrees();
        double currentAngle = -drivetrain.getPose().getRotation().getDegrees();

        return new TurnByEncoder((targetAngle - currentAngle) % 360 - 180, drivetrain);
    }
}
