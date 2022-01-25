/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * interface for homing a subsystem.
 */
public interface Homeable {
    public double getCurrent();

    public double getVelocity();

    public void setHomingPower(double power);

    public void setHome(double position);

    public void setSoftLimits(double revers, double forward);

    public void disableSoftLimits();

    public void enableSoftLimits();

    public Subsystem getSubsystemObject();
}
