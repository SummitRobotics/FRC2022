// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Cancel momentum gained from swinging
 */
public class CancelAngularMomentum extends CommandBase {

  private final Drivetrain drivetrain;
  private final AHRS gyro;
  private final double kPowerCoefficient = 1/90;
  private double lastPitch;

  /**
   * Cancel momentum gained from swinging
   * 
   * @param drivetrain
   * @param gyro
   */
  public CancelAngularMomentum(Drivetrain drivetrain, AHRS gyro) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.gyro = gyro;
  }

  @Override
  public void initialize() 
  {
    lastPitch = gyro.getPitch();
  }

  @Override
  public void execute() 
  {
    double deltaPitch = lastPitch - gyro.getPitch();
    drivetrain.setBothMotorPower(deltaPitch * kPowerCoefficient);
    lastPitch = gyro.getPitch();
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
