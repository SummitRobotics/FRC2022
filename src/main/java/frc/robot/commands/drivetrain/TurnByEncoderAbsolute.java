package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;

/**
 * Parallel Command to turn the robot using encoders (relative to 0, not the robot's position).
 */
public class TurnByEncoderAbsolute extends ParallelCommandGroup {

    // TODO - check accuracy
    private static final double ROBOT_RADIUS = Drivetrain.DRIVE_WIDTH / 2;

    /**
     * Constructor to turn the robot using encoders (relative to 0, not the robot's position).
     *
     * @param angle Angle to turn the robot.
     * @param drive Drivetrain subsystem.
     */
    public TurnByEncoderAbsolute(double angle, Drivetrain drive) {
        double radians = (Math.PI / 180) * angle;
        double distance = ROBOT_RADIUS * radians;
        addCommands(new EncoderDrive(distance - drive.encoderToDist(drive.getLeftEncoderPosition()),
            -distance + drive.encoderToDist(drive.getRightEncoderPosition()), drive));
    }
}
