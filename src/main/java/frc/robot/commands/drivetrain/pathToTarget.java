// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.PIDValues;
/**
 *  goes to a target using limelight. 
 */

public class pathToTarget extends CommandBase {
    /** Creates a new pathToTarget. */
    Lemonlight limelight;
    Drivetrain drivetrain;
    PIDController alignPID;
    PIDController movePID;
    boolean hasRecordedLimelightDistance;
    static int count;
    /**
     * Paths to a target using limelight data. 
     *
     * @param limelight limelight to use
     * @param drivetrain drivetrain for pathing
     */
    
    public pathToTarget(Lemonlight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignPID = new PIDController(PIDValues.ALIGN_P, PIDValues.ALIGN_I, PIDValues.ALIGN_D);
        this.movePID = new PIDController(PIDValues.MOVE_P, PIDValues.MOVE_I, PIDValues.MOVE_D);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        count = 0;
        alignPID.reset();
        movePID.reset();
        hasRecordedLimelightDistance = false;
        movePID.setSetpoint(0);
        alignPID.setSetpoint(0);
        movePID.setTolerance(1, 1);
        alignPID.setTolerance(1, 1);
        drivetrain.highGear();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            double angle = limelight.getVerticalOffset();
            double distance = limelight.getLimelightDistanceEstimateIN(limelight.MAIN_MOUNT_HEIGHT, limelight.MAIN_MOUNT_ANGLE, limelight.MAIN_TARGET_HEIGHT, angle);
            double movePower = -Functions.clampDouble(movePID.calculate(distance), 0.5, -0.5);
            double turnPower = alignPID.calculate(limelight.getHorizontalOffset());
            drivetrain.setLeftMotorPower(movePower - turnPower);
            drivetrain.setRightMotorPower(movePower + turnPower);
            count = 0;
        } else {
            count++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        count = 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (movePID.atSetpoint() && alignPID.atSetpoint()) || count > 150;
    }
}
