package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;

/**
 * Parallel Command to turn the robot using encoders.
 */
public class TurnByEncoder extends ParallelCommandGroup {

    //TODO check accuracy
    private static final double ROBOT_RADIUS = Drivetrain.DRIVE_WIDTH / 2;

    /**
     * Constructor to turn the robot using encoders, rotating clockwise.
     *
     * @param angle Angle to turn the robot.
     * @param drivetrain Drivetrain subsystem.
     */
    public TurnByEncoder(double angle, Drivetrain drivetrain) {
        double radians = (Math.PI / 180) * angle;
        double distance = ROBOT_RADIUS * radians;
        addCommands(new EncoderDrive(distance, -distance, drivetrain));
    }
}
