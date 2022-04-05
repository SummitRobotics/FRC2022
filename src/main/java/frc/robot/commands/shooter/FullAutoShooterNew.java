// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.Lemonlight.LEDModes;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.PIDValues;

public class FullAutoShooterNew extends CommandBase {

    private Conveyor conveyor;
    private Shooter shooter;
    private Drivetrain drivetrain;
    private Lemonlight lemonlight;

    private final double[] waypoints = {65, 155};

    private final double maxDist = 160;
    private final double hoodDistCutoff = 125; 
    private final double minDist = 50;

    private final double moveError = 3;
    private final double okToMoveError = 15;
    private final double alignError = 2;
    private final double speedError = 40;
    private final double okToSpoolError = 25;

    private final double alignOffset = 14;
    private final double idleSpeed = 1700;


    //not tunable
    private PIDController alignPid;
    private PIDController movePid;

    /** Creates a new FullAutoShooterNew. */
    public FullAutoShooterNew(Drivetrain drivetrain, Shooter shooter, Conveyor conveyor, Lemonlight lemonlight) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.lemonlight = lemonlight;

        alignPid = new PIDController(PIDValues.ALIGN_P, PIDValues.ALIGN_I, PIDValues.ALIGN_D);
        movePid = new PIDController(PIDValues.MOVE_P, PIDValues.MOVE_I, PIDValues.MOVE_D);
        alignPid.setTolerance(alignError);
        movePid.setTolerance(moveError);

    }

    @Override
    public void initialize() {
        alignPid.reset();
        movePid.reset();
        lemonlight.setLEDMode(LEDModes.FORCE_ON);
    }

    @Override
    public void execute() {
        if (!lemonlight.hasTarget()) {
            noTarget();
        } else {
            double lld  = Lemonlight.getLimelightDistanceEstimateIN(
                Lemonlight.MAIN_MOUNT_HEIGHT,
                Lemonlight.MAIN_MOUNT_ANGLE,
                Lemonlight.MAIN_TARGET_HEIGHT,
                lemonlight.getVerticalOffset());

            shooter.setHoodPos(lld > hoodDistCutoff);

            boolean aligned = alignAndDrive(lld);

            double rpm = 0;

            if (Functions.isWithin(lld, movePid.getSetpoint(), okToSpoolError)) {
                rpm = solveMotorSpeed(lld, shooter.getHoodPos());
            } else {
                rpm = idleSpeed;
            }

            shooter.setMotorTargetSpeed(rpm);


            if (aligned && Functions.isWithin(shooter.getShooterRPM(), rpm, speedError) && okToShoot()) {
                shooter.setState(Shooter.States.READY_TO_FIRE);
            } else {
                shooter.setState(Shooter.States.NOT_SHOOTING);
            }

        }

    }

    //makes semiAuto super simple to impliment
    private boolean okToShoot() {
        return true;
    }


    /**
     * Returns the desired motor speed based on the distance from the target and the hood position.
     *
     * @param distance The distance from the target.
     * @param hoodPos The hood position.
     * @return motorSpeed The desired motor speed
     */
    public double solveMotorSpeed(double distance, boolean hoodPos) {
        // Annoyingly, exponent notation requires importing a math library.
        // So, for simplicity, we do not use it.
        // TODO - Test the shooter and do a regression to find the right formula.
        // Add higher order terms if necessary.
        if (hoodPos) {
            //System.out.println("reg down");
            return
                (-0.00330099 * distance * distance * distance)
                + (1.44904 * distance * distance)
                + (-205.058 * distance)
                + 11204.6;
        } else {
            //System.out.println("reg up");
            return 
                (-0.00000243868 * distance * distance * distance * distance * distance)
                + (0.000923236 * distance * distance * distance * distance)
                + (-0.131588 * distance * distance * distance)
                + (8.76388 * distance * distance)
                + (-266.098 * distance)
                + 4428.38;
        }
    }

    private boolean alignAndDrive(double lld) {
        double ofset = lemonlight.getHorizontalOffset();

        double leftPower = 0;
        double rightPower = 0;

        movePid.setSetpoint(Functions.findClosestPoint(lld, waypoints));

        if (movePid.atSetpoint()) {
            //math to keep offset from target center constant ad dist changes
            alignPid.setSetpoint(Math.toDegrees(Math.atan(alignOffset / lld)));
        } else {
            alignPid.setSetpoint(0);
        }

        if (Functions.isWithin(ofset, 0, okToMoveError)) {
            double power = movePid.calculate(lld);
            leftPower -= power;
            rightPower -= power;
        }

        double alignPower = alignPid.calculate(ofset);

        leftPower -= alignPower;
        rightPower += alignPower;

        return movePid.atSetpoint() && alignPid.atSetpoint();      
    }

    private void noTarget() {
        drivetrain.setLeftMotorPower(0.3);
        drivetrain.setRightMotorPower(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        shooter.stop();
        shooter.setState(Shooter.States.NOT_SHOOTING);
        lemonlight.setLEDMode(LEDModes.FORCE_OFF);
    }

    @Override
    public boolean isFinished() {
        return !conveyor.doesBallExist();
    }
}
