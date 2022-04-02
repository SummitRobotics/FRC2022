/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to move the drivetrain with the encoders.
 */
public class TurnByEncoderAbsolute extends CommandBase {

    private static final double ROBOT_RADIUS = Drivetrain.DRIVE_WIDTH / 2;

    private final Drivetrain drivetrain;
    private final Rotation2d direction;

    private double left = Integer.MAX_VALUE;
    private double right = Integer.MAX_VALUE;

    private final double maximumPower = 0.5;
    /**
     * The Constructor.
     *
     * @param drivetrain the robot's drivetrain
     * @param direction the direction to point the robot
     */
    public TurnByEncoderAbsolute(Rotation2d direction, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.direction = direction;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.stop();
        drivetrain.setPIDMaxPower(maximumPower);

        double targetAngle = -direction.getDegrees();
        double currentAngle = -drivetrain.getPose().getRotation().getDegrees();

        double angle = targetAngle - currentAngle;

        // Limits angle to +- 180 so robot doesn't turn more than it needs to.
        angle -= Math.ceil(angle / 360 - 0.5) * 360;

        double radians = (Math.PI / 180) * angle;
        double distance = ROBOT_RADIUS * radians;

        // TODO test if this is accurate
        left = drivetrain.getLeftDistance() + distance;
        drivetrain.setLeftMotorTarget(drivetrain.distToEncoder(left));

        right = drivetrain.getRightDistance() - distance;
        drivetrain.setRightMotorTarget(drivetrain.distToEncoder(right));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getLeftDistance() - left) < 0.05
            && Math.abs(drivetrain.getRightDistance() - right) < 0.05;
    }
}
