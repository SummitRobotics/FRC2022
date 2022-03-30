/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to move the drivetrain with the encoders.
 */
public class EncoderDrive extends CommandBase {

    private final Drivetrain drivetrain;
    private final double left;
    private final double right;

    /**
     * The Constructor.
     *
     * @param drivetrain the robot's drivetrain
     * @param left The distance to move the left side
     * @param right The distance to move the right side
     */
    public EncoderDrive(double left, double right, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.stop();
        // TODO test if this is accurate
        drivetrain.setLeftMotorTarget(drivetrain.distToEncoder(left)
            + drivetrain.getLeftEncoderPosition());
        drivetrain.setRightMotorTarget(drivetrain.distToEncoder(right)
            + drivetrain.getRightEncoderPosition());
    }

    @Override
    public void execute() {
        System.out.println(drivetrain.getLeftDistance());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getLeftDistance() - left) < 0.01
            && Math.abs(drivetrain.getRightDistance() - right) < 0.01;

    }
}
