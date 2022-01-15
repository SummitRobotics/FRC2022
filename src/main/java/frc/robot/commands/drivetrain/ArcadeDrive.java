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

public class ArcadeDrive extends CommandBase {

    private Drivetrain drivetrain;

    private OIAxis forwardPowerAxis;
    private OIAxis reversePowerAxis;
    private OIAxis turnAxis;
    private ChangeRateLimiter limiter;

    private final double deadzone = .1;

    private double max_change_rate = 0.1;
    
    /**
     * teleop driver control
     * @param drivetrain drivetrain instance
     * @param shift shifter instance
     * @param powerAxis control axis for the drivetrain power
     * @param turnAxis control axis for the drivetrain turn
     */
    public ArcadeDrive(
        Drivetrain drivetrain, 
        OIAxis forwardPowerAxis, 
        OIAxis reversePowerAxis, 
        OIAxis turnAxis)
    {

        this.drivetrain = drivetrain;

        this.forwardPowerAxis = forwardPowerAxis;
        this.reversePowerAxis = reversePowerAxis;
        this.turnAxis = turnAxis;

        limiter = new ChangeRateLimiter(max_change_rate);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setOpenRampRate(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double forwardPower = forwardPowerAxis.get();
        double reversePower = reversePowerAxis.get();

        forwardPower = Functions.deadzone(deadzone, forwardPower);
        reversePower = Functions.deadzone(deadzone, reversePower);

        forwardPower = Math.pow(forwardPower, 2);
        reversePower = Math.pow(reversePower, 2);

        double power = forwardPower - reversePower;
        double turn = Math.pow(turnAxis.get(), 3);

        power = limiter.getRateLimitedValue(power);

        // calculates power to the motors
        double leftPower = power + turn;
        double rightPower = power - turn;

        drivetrain.setLeftMotorPower(leftPower);
        drivetrain.setRightMotorPower(rightPower);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
