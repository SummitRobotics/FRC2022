/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;

/**
 * Command for Arcade Drive.
 */
public class ArcadeDrive extends CommandBase {

    private final Drivetrain drivetrain;

    private final OIAxis forwardPowerAxis;
    private OIAxis reversePowerAxis;
    private final OIAxis turnAxis;

    private final ChangeRateLimiter limiter;

    private static final double DEAD_ZONE = .1;

    private static final double MAX_CHANGE_RATE = 0.05;

    private final RollingAverage avgSpeed = new RollingAverage(2, true);

    private final RollingAverage avgPower = new RollingAverage(2, true);

    private final boolean isSingleAxis;

    /**
     * teleop driver control.
     *
     * @param drivetrain       drivetrain instance
     * @param forwardPowerAxis control axis for forward power
     * @param reversePowerAxis control axis for reverse power
     * @param turnAxis         control axis for the drivetrain turn
     */
    public ArcadeDrive(
        Drivetrain drivetrain, 
        OIAxis forwardPowerAxis, 
        OIAxis reversePowerAxis, 
        OIAxis turnAxis) {

        this.drivetrain = drivetrain;

        this.forwardPowerAxis = forwardPowerAxis;
        this.reversePowerAxis = reversePowerAxis;
        this.turnAxis = turnAxis;

        limiter = new ChangeRateLimiter(MAX_CHANGE_RATE);

        addRequirements(drivetrain);
        isSingleAxis = false;
    }

    /**
     * teleop driver control.
     *
     * @param drivetrain drivetrain instance
     * @param powerAxis  control axis for the drivetrain power
     * @param turnAxis   control axis for the drivetrain turn
     */
    public ArcadeDrive(
        Drivetrain drivetrain, 
        OIAxis powerAxis, 
        OIAxis turnAxis) {

        this.drivetrain = drivetrain;

        this.forwardPowerAxis = powerAxis;
        this.turnAxis = turnAxis;

        limiter = new ChangeRateLimiter(MAX_CHANGE_RATE);

        addRequirements(drivetrain);
        isSingleAxis = true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setOpenRampRate(0);
        avgPower.reset();
        avgSpeed.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double power;

        if (isSingleAxis) {
            power = Math.pow(Functions.deadzone(DEAD_ZONE, forwardPowerAxis.get()), 3);
        } else {
            double forwardPower = forwardPowerAxis.get();
            double reversePower = reversePowerAxis.get();

            forwardPower = Functions.deadzone(DEAD_ZONE, forwardPower);
            reversePower = Functions.deadzone(DEAD_ZONE, reversePower);

            forwardPower = Math.pow(forwardPower, 2);
            reversePower = Math.pow(reversePower, 2);

            power = forwardPower - reversePower;
        }

        power = limiter.getRateLimitedValue(power);

        avgPower.update(Math.abs(power));

        avgSpeed.update(Math.abs(drivetrain.getRightRPM()) + Math.abs(drivetrain.getLeftRPM()));

        //shifts into low gear if drivetrain stalled
        if ((avgPower.getAverage() > .5) && avgSpeed.getAverage() < 15) {
            drivetrain.lowGear();
        }

        double turn = Math.pow(turnAxis.get(), 3);


        // calculates power to the motors
        double leftPower = power + turn;
        double rightPower = power - turn;

        drivetrain.setLeftMotorPower(leftPower);
        drivetrain.setRightMotorPower(rightPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
