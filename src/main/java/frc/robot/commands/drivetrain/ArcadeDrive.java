/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.SimpleButton;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;

/**
 * Command for Arcade Drive.
 */
public class ArcadeDrive extends CommandBase {

    private final Drivetrain drivetrain;
    private final Intake intake;

    private double forwardPower;
    private double reversePower;
    private boolean activateSwitchfoot;

    private final OIAxis.PrioritizedAxis forwardPowerAxis;
    private final OIAxis.PrioritizedAxis reversePowerAxis;
    private final OIButton.PrioritizedButton switchfootPRI;
    private final SimpleButton switchfoot;
    private final OIAxis.PrioritizedAxis turnAxis;

    private final ChangeRateLimiter limiter;
    private final ChangeRateLimiter turnLimiter;


    private static final double DEAD_ZONE = .01;

    private static final double MAX_CHANGE_RATE = 0.05;

    private final RollingAverage avgSpeed = new RollingAverage(2, true);

    private final RollingAverage avgPower = new RollingAverage(2, true);

    private final boolean isSingleAxis;

    private boolean ledsOn = false;

    /**
     * teleop driver control.
     *
     * @param drivetrain       drivetrain instance
     * @param forwardPowerAxis control axis for forward power
     * @param reversePowerAxis control axis for reverse power
     * @param turnAxis         control axis for the drivetrain turn
     * @param switchfoot         drive in reverse
     */
    public ArcadeDrive(
        Drivetrain drivetrain,
        Intake intake, 
        OIAxis forwardPowerAxis, 
        OIAxis reversePowerAxis, 
        OIAxis turnAxis,
        OIButton switchfoot) {
        switchfootPRI = switchfoot.prioritize(AxisPriorities.DRIVE);
        this.switchfoot = new SimpleButton(switchfootPRI::get);
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.forwardPowerAxis = forwardPowerAxis.prioritize(AxisPriorities.DRIVE);
        this.reversePowerAxis = reversePowerAxis.prioritize(AxisPriorities.DRIVE);
        this.turnAxis = turnAxis.prioritize(AxisPriorities.DRIVE);

        limiter = new ChangeRateLimiter(MAX_CHANGE_RATE);
        turnLimiter = new ChangeRateLimiter(0.05);


        addRequirements(drivetrain);
        isSingleAxis = false;
        ledsOn = false;
    }

    /**
     * teleop driver control.
     *
     * @param drivetrain drivetrain instance
     * @param powerAxis  control axis for the drivetrain power
     * @param turnAxis   control axis for the drivetrain turn
     * @param switchfoot swaps direction
     */
    public ArcadeDrive(
        Drivetrain drivetrain, 
        Intake intake,
        OIAxis powerAxis, 
        OIAxis turnAxis, 
        OIButton switchfoot) {
        switchfootPRI = switchfoot.prioritize(AxisPriorities.DRIVE);
        this.switchfoot = new SimpleButton(switchfootPRI::get);
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.forwardPowerAxis = powerAxis.prioritize(AxisPriorities.DRIVE);
        this.reversePowerAxis = null;
        this.turnAxis = turnAxis.prioritize(AxisPriorities.DRIVE);

        limiter = new ChangeRateLimiter(MAX_CHANGE_RATE);
        turnLimiter = new ChangeRateLimiter(0.05);

        addRequirements(drivetrain);
        isSingleAxis = true;
        ledsOn = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setOpenRampRate(0);
        avgPower.reset();
        avgSpeed.reset();
        activateSwitchfoot = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double power;

        if (isSingleAxis) {
            power = Math.pow(Functions.deadzone(DEAD_ZONE, forwardPowerAxis.get()), 3);
        } else {
            forwardPower = forwardPowerAxis.get();
            reversePower = reversePowerAxis.get();   

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

        //System.out.println(turn);

        double limiterTurn = turnLimiter.getRateLimitedValue(turn);

        // if (intake.getState() == Intake.States.DOWN) {
        //     turn = limiterTurn;
        // }

        if (activateSwitchfoot) {
            //dumb stuff is dumb
            if (!ledsOn) {
                LEDs.getInstance().addCall("reversed", new LEDCall(LEDPriorities.DRIVE_REV, LEDRange.Aarms).ffh(Colors.YELLOW, Colors.OFF));   
            }
            turn = -turn;
        }


        // calculates power to the motors
        double leftPower = power + turn;
        double rightPower = power - turn;

        if (switchfoot.get()) {
            activateSwitchfoot = !activateSwitchfoot;
            if (activateSwitchfoot) {
                LEDs.getInstance().addCall("reversed", new LEDCall(LEDPriorities.DRIVE_REV, LEDRange.Aarms).ffh(Colors.YELLOW, Colors.OFF));  
            } else {
                LEDs.getInstance().removeCall("reversed");
            }
        }

        if (!activateSwitchfoot) {
            drivetrain.setLeftMotorPower(leftPower);
            drivetrain.setRightMotorPower(rightPower);
        } else {
            drivetrain.setLeftMotorPower(-leftPower);
            drivetrain.setRightMotorPower(-rightPower);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        LEDs.getInstance().removeCall("reversed");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}