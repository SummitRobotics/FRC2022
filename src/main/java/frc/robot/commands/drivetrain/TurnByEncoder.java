package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

/**
 * Parallel Command to turn the robot using encoders.
 */
public class TurnByEncoder extends ParallelCommandGroup {

    private static final double ROBOT_RADIUS = 15;

    /**
     * Constructor to turn the robot using encoders.
     *
     * @param angle Angle to turn the robot.
     * @param drive Drivetrain subsystem.
     * @param shift Shifter subsystem.
     */
    public TurnByEncoder(double angle, Drivetrain drive, Shifter shift) {
        double radians = Math.PI / 180;
        double distance = ROBOT_RADIUS * radians;
        addCommands(new MoveByDistance(distance, -distance, drive, shift));
    }
}
