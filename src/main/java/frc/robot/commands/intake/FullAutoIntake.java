package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.CustomVisionData;
import java.util.List;

/**
 * Full auto intake mode.
 */
public class FullAutoIntake extends CommandBase {

    // subsystems
    private Drivetrain drivetrain;

    // devices
    private Lemonlight ballDetectionLimelight;

    // pathfinding objects
    private FollowTrajectory followTrajectoryCommand;
    private Pose2d ballPose = new Pose2d();
    private Trajectory trajectory;

    /**
     * Constructor.
     *
     * @param drivetrain The drivetrain subsystem
     * @param ballDetectionLimelight The ball detection limelight
     */
    public FullAutoIntake(Drivetrain drivetrain,
        Lemonlight ballDetectionLimelight) {

        this.drivetrain = drivetrain;
        this.ballDetectionLimelight = ballDetectionLimelight;
    }

    @Override
    public void initialize() {
        // Gets the best ball.
        CustomVisionData ball = getBestBall();

        if (ball.isValid()) {

            double angle = Units.degreesToRadians(ball.getXAngle());

            ballPose = new Pose2d(
                Units.inchesToMeters(ball.getDistance()) * Math.cos(angle),
                Units.inchesToMeters(ball.getDistance()) * -Math.sin(angle),
                Rotation2d.fromDegrees(0)
            );


            TrajectoryConfig config = new TrajectoryConfig(2.5, 2.5)
                .setKinematics(Drivetrain.DriveKinimatics)
                .addConstraint(drivetrain.getVoltageConstraint());

            trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, new Rotation2d(0)), ballPose),
                config);

            followTrajectoryCommand = new FollowTrajectory(drivetrain,
                trajectory.relativeTo(drivetrain.getPose()));
        }
    }

    @Override
    public void execute() {
        if (trajectory == null) {
            System.out.println("The trajectory is null!");
        } else {
            followTrajectoryCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return followTrajectoryCommand == null
            || followTrajectoryCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (followTrajectoryCommand != null) {
            followTrajectoryCommand.end(interrupted);
        }
    }

    private CustomVisionData getBestBall() {
        return CustomVisionData.calculateBestBall(
            Lemonlight.parseVisionData(ballDetectionLimelight.getCustomVisionData()), true);
    }
}
