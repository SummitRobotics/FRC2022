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

    private final double wheelDiameter = 6;
    private final double lowGearRatio = 19.61 / 1;
    private final double highGearRatio = 9.07 / 1;

    private Drivetrain drive;
    private Shifter shifter;

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
            rightRotations = ((wheelDiameter * Math.PI) * right) * highGearRatio;
            leftRotations = (wheelDiameter * Math.PI) * highGearRatio * left;

        } else {
            rightRotations = (wheelDiameter * Math.PI) * lowGearRatio * right;
            leftRotations = (wheelDiameter * Math.PI) * lowGearRatio * left;
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
