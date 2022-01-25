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
     * @param left The number of motor rotations to move the left side
     * @param right The number of motor rotations to move the right side
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
        drivetrain.zeroDistance();
        // TODO test if this is acurate
        drivetrain.setLeftMotorTarget(drivetrain.distToEncoder(left));
        drivetrain.setRightMotorTarget(drivetrain.distToEncoder(right));
    }

    @Override
    public void execute() {
        // System.out.println(drivetrain.getLeftEncoderPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getLeftDistance() - left) < 1
            && Math.abs(drivetrain.getRightDistance() - right) < 1;

    }
}
