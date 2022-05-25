// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.oi.inputs.OIButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.PIDValues;

public class ProtectedShooter extends CommandBase {
    /** Creates a new ProtectedShooter. */
    private final double angleOffset = 7.5;
    private Conveyor conveyor;
    private Shooter shooter;
    private Drivetrain drivetrain;
    private Lemonlight limelight;
    private OIButton trigger;
    private OIButton.PrioritizedButton prioritizedTrigger;
    RollingAverage avg = new RollingAverage(5, false);
    protected PIDController alignPID;
    private double speed = 1850;
    private double error = 25;

    public ProtectedShooter(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        Lemonlight limelight,
        OIButton trigger) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.trigger = trigger;
        this.limelight = limelight;
        this.alignPID = new PIDController(PIDValues.ALIGN_P, PIDValues.ALIGN_I, PIDValues.ALIGN_D);

        alignPID.setTolerance(2, 9999999);
        alignPID.setSetpoint(angleOffset);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain, shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        prioritizedTrigger = trigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
    }

    /**
     *  Aligns to target.
     *
     * @param drivetrain drivetrain 
     * @param horizontalOffset horizontal offset to angle to
     * @param limelightDistanceEstimate distance
     * @return if aligned
     */
    
    public boolean driveAndAlign(Drivetrain drivetrain, double horizontalOffset, double limelightDistanceEstimate) {
    
        //sets if align target based on ball color
        // if (isAccurate) {
        //     alignPID.setSetpoint(0);
        // } else {
        //     //alignPID.setSetpoint(0);
        //     alignPID.setSetpoint(TARGET_WRONG_COLOR_MISS);
        // }

        alignPID.setSetpoint(angleOffset);
        
        double leftPower = 0;
        double rightPower = 0;

        //align to target
        double alignPower = alignPID.calculate(horizontalOffset);

        leftPower += -alignPower;
        rightPower += alignPower;


        //sets drivetrain powers
        drivetrain.setLeftMotorPower(leftPower);
        drivetrain.setRightMotorPower(rightPower);
        System.out.println("Is aligned " + alignPID.atSetpoint());
        return alignPID.atSetpoint();
    }

    @Override
    public void execute() {
        // shooter.setMotorPower(controlAxis.get());
        shooter.setHoodPos(false);
        avg.update(shooter.getShooterRPM());
        double limelightDistanceEstimate = Units.metersToInches(limelight.getLimelightDistanceEstimate());
        shooter.setHoodPos(true);
        double horizontalOffset = Units.radiansToDegrees(limelight.getHorizontalOffset());
        boolean aligned = driveAndAlign(drivetrain, horizontalOffset, limelightDistanceEstimate);

        if (Functions.isWithin(avg.getAverage(), speed, error) && aligned && prioritizedTrigger.get()) {
            shooter.setState(Shooter.States.READY_TO_FIRE);
            // System.out.println("rpm: " + shooter.getShooterRPM());
        } else {
            shooter.setState(Shooter.States.NOT_SHOOTING);
        }
        //shooter.setMotorVolts(shooter.calculateVoltageFromPid(1700));
        shooter.setMotorTargetSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !conveyor.doesBallExist();
    }
}
