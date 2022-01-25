// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.homing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.Homeable;

/**
 * Homes a subsystem with encoders.
 */
public class HomeByEncoder extends CommandBase {

    private final boolean limits;
    private final Homeable toHome;
    private final double homingPower;
    private final int minLoops;
    private double reverseLimit;
    private double forwardLimit;

    private int loops;

    /**
     * Constructor.
     */
    public HomeByEncoder(Homeable toHome, double homingPower, int minLoops) {
        this.toHome = toHome;
        this.homingPower = homingPower;
        this.minLoops = minLoops;

        loops = 0;

        limits = false;

        addRequirements(toHome.getSubsystemObject());
    }

    /**
     * Constructor.
     */
    public HomeByEncoder(
            Homeable toHome,
            double homingPower,
            int minLoops,
            double reverseLimit,
            double forwardLimit) {
        this.toHome = toHome;
        this.homingPower = homingPower;
        this.minLoops = minLoops;
        this.reverseLimit = reverseLimit;
        this.forwardLimit = forwardLimit;

        limits = true;

        loops = 0;

        addRequirements(toHome.getSubsystemObject());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        loops = 0;
        toHome.disableSoftLimits();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        toHome.setHomingPower(homingPower);
        loops += (loops > minLoops) ? 0 : 1;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        toHome.setHomingPower(0);
        if (!interrupted) {
            toHome.setHome(0);
            if (limits) {
                toHome.setSoftLimits(reverseLimit, forwardLimit);
                toHome.enableSoftLimits();
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double v = toHome.getVelocity();

        return (loops > minLoops) && (Math.abs(v) < 1);
    }
}
