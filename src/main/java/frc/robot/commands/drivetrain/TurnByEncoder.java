package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

public class TurnByEncoder extends ParallelCommandGroup {

    private static final double ROBOT_RADIUS = 15;

    public TurnByEncoder(double angle, Drivetrain drive, Shifter shift) {
        double radians = Math.PI / 180;
        double distance = ROBOT_RADIUS * radians;
        addCommands(new MoveByDistance(distance, -distance, drive, shift));
    }
}
