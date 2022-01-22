/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;
import frc.robot.utilities.Functions;

public class MoveByDistance extends CommandBase {

    private double left, right;
    private double leftRotations, rightRotations;

    // TODO move to drivetrain
    private static final double WHEEL_DIAMETER = 6;

    private Drivetrain drive;
    private Shifter shifter;

    // TODO probably rewright this whole thing to use the spline system
    public MoveByDistance(double left, double right, Drivetrain drivetrain, Shifter shift) {
        drive = drivetrain;
        shifter = shift;

        this.left = left;
        this.right = right;

        addRequirements(drivetrain, shifter);
    }

    public MoveByDistance(double angle, Drivetrain drivetrain, Shifter shift) {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (shifter.getShiftState()) {
            rightRotations = ((WHEEL_DIAMETER * Math.PI) * right) * drive.HIGH_GEAR_RATIO;
            leftRotations = (WHEEL_DIAMETER * Math.PI) * drive.HIGH_GEAR_RATIO * left;

        } else {
            rightRotations = (WHEEL_DIAMETER * Math.PI) * drive.LOW_GEAR_RATIO * right;
            leftRotations = (WHEEL_DIAMETER * Math.PI) * drive.LOW_GEAR_RATIO * left;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.setLeftMotorTarget(leftRotations);
        drive.setRightMotorTarget(rightRotations);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.setLeftMotorPower(0);
        drive.setRightMotorPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Functions.isWithin(drive.getLeftEncoderPosition(), leftRotations, 1)
                && Functions.isWithin(drive.getRightEncoderPosition(), rightRotations, 1);
    }
}
