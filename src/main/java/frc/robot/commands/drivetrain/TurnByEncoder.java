package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class TurnByEncoder extends ParallelCommandGroup {

	//TODO check acuracy
	private final double robotRadius = 15;

	public TurnByEncoder(double angle, Drivetrain drive) {
		double radians = Math.PI / 180;
		double distance = robotRadius * radians;
		addCommands(new EncoderDrive(distance, -distance, drive));
	}
}
