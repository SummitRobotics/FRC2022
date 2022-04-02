package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.FollowDynamicTrajectoryThreaded;
import frc.robot.commands.drivetrain.FollowTrajectoryThreaded;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.commands.drivetrain.TurnByEncoderAbsolute;
import frc.robot.subsystems.Drivetrain;
import java.util.List;
import java.util.function.Supplier;

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
        }, drivetrain));

        this.drivetrain = drivetrain;
        this.config = drivetrain.getTrajectoryConfigHighGear();
    }

    protected Command moveTo(Pose2d end) {
        return moveTo(() -> end);
    }

    protected Command moveTo(Supplier<Pose2d> end) {
        return moveTo(end, List::of);
    }

    protected Command moveTo(Pose2d end, List<Translation2d> midpoints) {
        return moveTo(() -> end, () -> midpoints);
    }

    protected Command moveTo(Supplier<Pose2d> end, Supplier<List<Translation2d>> midpoints) {
        return moveTo(drivetrain::getPose, end, midpoints);
    }

    private Command moveTo(Supplier<Pose2d> start, Supplier<Pose2d> end, Supplier<List<Translation2d>> midpoints) {
        return new FollowDynamicTrajectoryThreaded(start, end, midpoints, config, drivetrain);
    }

    protected Command rotateTo(Rotation2d direction) {
        return new TurnByEncoderAbsolute(direction, drivetrain);
    }
}
