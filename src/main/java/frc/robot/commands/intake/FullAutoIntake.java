package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.FollowTrajectoryThreaded;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.CustomVisionData;
import java.util.Arrays;

/**
 * Full auto intake mode.
 */
public class FullAutoIntake extends CommandBase {

    private Drivetrain drivetrain;
    private Lemonlight ballDetectionLimelight;
    private FollowTrajectory followTrajectoryCommand;
    private Pose2d ballPose = new Pose2d();

    /**
     * Constructor.
     *
     * @param drivetrain the robot drivetrain
     * @param ballDetectionLimelight the ball detection limelight
     */
    //FullAutoIntake(Drivetrain drivetrain, Intake intake, Lemonlight ballDetectionLimelight) {
    public FullAutoIntake(Drivetrain drivetrain, Lemonlight ballDetectionLimelight) {
        this.drivetrain = drivetrain;
        this.ballDetectionLimelight = ballDetectionLimelight;
    }

    @Override
    public void initialize() {
        // Gets the best ball.
        CustomVisionData ball = getBestBall();

        if (ball.isValid()) {

            // Generates the poses.
            //Pose2d startPosition = drivetrain.getPose();
            //ballPose = calculateBallPose(ball, drivetrain.getPose());

            Pose2d startPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            ballPose =
                    new Pose2d(Units.inchesToMeters(ball.getYDistance() + 12), -Units.inchesToMeters(ball.getXDistance()), Rotation2d.fromDegrees(0));

            drivetrain.setPose(startPosition);

            // Generates the trajectory's
            TrajectoryConfig config = new TrajectoryConfig(4, 2.5);
            System.out.println("Ball Position" + ballPose);
            Trajectory trajectory = TrajectoryGenerator
                    .generateTrajectory(Arrays.asList(startPosition, ballPose), config);

            // Creates the command to follow the ball.
            followTrajectoryCommand = new FollowTrajectory(drivetrain, trajectory);
            followTrajectoryCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (followTrajectoryCommand != null) {
//            Pose2d newBallPose = calculateBallPose(getBestBall(), drivetrain.getPose());
//            if (!inProximity(newBallPose, ballPose, 0.2)) {
//                //System.out.println("NEW POSITION: " + newBallPose);
//                ballPose = newBallPose;
//            }

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

    private Pose2d calculateBallPose(CustomVisionData bestBall, Pose2d drivetrainPose) {

        double angle = drivetrainPose.getRotation().getRadians()
                + Units.degreesToRadians(bestBall.getXAngle());

        // Ball Position relative to field with robot at origin.
        Pose2d ball = new Pose2d(
            Units.inchesToMeters(bestBall.getDistance()) * Math.cos(angle),
            Units.inchesToMeters(bestBall.getDistance()) * Math.sin(angle) * -1,
            Rotation2d.fromDegrees(0)
            );

        // Vector to translate the ball to move it properly.
        Translation2d ballTranslation =
                new Translation2d(drivetrainPose.getX(), drivetrainPose.getY());

        // Ball transform. Basically translation with angle.
        Transform2d ballTransform =
                new Transform2d(ballTranslation, drivetrainPose.getRotation());
        return ball.plus(ballTransform);
    }

    private CustomVisionData getBestBall() {
        return CustomVisionData.calculateBestBall(
            Lemonlight.parseVisionData(ballDetectionLimelight.getCustomVisionData()), true);
    }

    private boolean inProximity(Pose2d pos1, Pose2d pos2, double distance) {
        double dx = pos1.getX() - pos2.getX();
        double dy = pos1.getY() - pos2.getY();
        return Math.sqrt(dx * dx + dy * dy) <= distance;
    }
}
