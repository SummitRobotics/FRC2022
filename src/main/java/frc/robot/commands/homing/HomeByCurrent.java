/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.homing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.Homeable;
import frc.robot.utilities.RollingAverage;

/**
 * Class which allows a subsystem to home a mechanism by looking for current spikes.
 */
public class HomeByCurrent extends CommandBase {

    private final boolean limits;

    private final Homeable toHome;
    private final double homingPower;
    private final double currentThreshold;
    private double reverseLimit;
    private double forwardLimit;

    private final RollingAverage currentAverage = new RollingAverage(10, false);

    /**
     * Creates a new HomeByCurrent.
     */
    public HomeByCurrent(Homeable toHome, double homingPower, double currentThreshold) {
        this.toHome = toHome;
        this.homingPower = homingPower;
        this.currentThreshold = currentThreshold;

        limits = false;

        addRequirements(toHome.getSubsystemObject());
    }

    /**
     * The Constructor.
     *
     * @param toHome TODO
     * @param homingPower TODO
     * @param currentThreshold TODO
     * @param reversLimit TODO
     * @param forwardLimit TODO
     */
    public HomeByCurrent(
            Homeable toHome,
            double homingPower,
            double currentThreshold,
            double reversLimit,
            double forwardLimit) {
        this.toHome = toHome;
        this.homingPower = homingPower;
        this.currentThreshold = currentThreshold;
        this.reverseLimit = reversLimit;
        this.forwardLimit = forwardLimit;

        limits = true;

        addRequirements(toHome.getSubsystemObject());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentAverage.reset();

        // System.out.println("running");
        toHome.disableSoftLimits();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("homing current: " + toHome.getCurrent());
        toHome.setHomingPower(homingPower);
        currentAverage.update(toHome.getCurrent());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        toHome.setHomingPower(0);
        // prints out homing completed message
        System.out.println(
                "homing of "
                        + toHome.getSubsystemObject().getClass().getCanonicalName()
                        + " ended with interrupted "
                        + interrupted);
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
        double current = currentAverage.getAverage();
        return current >= currentThreshold;
    }
}
