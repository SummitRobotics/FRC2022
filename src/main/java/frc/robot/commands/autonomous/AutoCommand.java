package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FollowDynamicTrajectoryThreaded;
import frc.robot.commands.drivetrain.TurnByEncoderAbsolute;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Class for AutoCommands.
 */
public abstract class AutoCommand extends SequentialCommandGroup {
    protected final Drivetrain drivetrain;
    protected final TrajectoryConfig configForward;
    protected final TrajectoryConfig configReverse;

    AutoCommand(Drivetrain drivetrain, Pose2d startingPosition) {

        super(new InstantCommand(() -> {
            drivetrain.highGear();
            drivetrain.setPose(startingPosition);
        }, drivetrain));

        this.drivetrain = drivetrain;
        this.configForward = drivetrain.generateTrajectoryConfigHighGear();
        this.configReverse = drivetrain.generateTrajectoryConfigHighGear().setReversed(true);
    }

    public enum Direction {
        Forward,
        Reverse
    }

    protected Command moveTo(Pose2d end, Direction direction) {
        return moveToNondirectional(end, List.of(), direction);
    }

    protected Command moveToDirectional(Pose2d end, List<Pose2d> midpoints, Direction direction) {
        return moveToDirectional(drivetrain::getPose, () -> end, () -> midpoints, direction);
    }

    protected Command moveToNondirectional(Pose2d end, List<Translation2d> midpoints, Direction direction) {
        return moveToNondirectional(drivetrain::getPose, () -> end, () -> midpoints, direction);
    }

    private Command moveToNondirectional(Supplier<Pose2d> start, Supplier<Pose2d> end, Supplier<List<Translation2d>> midpoints, Direction direction) {
        TrajectoryConfig config = direction == Direction.Forward ? configForward : configReverse;
        return new FollowDynamicTrajectoryThreaded(start, end, midpoints, config, drivetrain);
    }

    private Command moveToDirectional(Supplier<Pose2d> start, Supplier<Pose2d> end, Supplier<List<Pose2d>> midpoints, Direction direction) {
        TrajectoryConfig config = direction == Direction.Forward ? configForward : configReverse;
        Supplier<List<Pose2d>> path = () -> {
            List<Pose2d> finalPath = new ArrayList<>();
            finalPath.add(start.get());
            finalPath.addAll(midpoints.get());
            finalPath.add(end.get());
            return finalPath;
        };
        return new FollowDynamicTrajectoryThreaded(path, config, drivetrain);
    }

    protected Command rotateTo(Rotation2d direction) {
        return new TurnByEncoderAbsolute(direction, drivetrain);
    }
}
