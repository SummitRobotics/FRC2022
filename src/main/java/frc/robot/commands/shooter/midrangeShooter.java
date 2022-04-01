package frc.robot.commands.shooter;

import java.sql.RowId;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.Lemonlight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.PIDValues;

/**
 * Manual override for the shooter.
 */
public class midrangeShooter extends CommandBase {

    Shooter shooter;

    Conveyor conveyor;
    Lemonlight limelight;
    Drivetrain drivetrain;
    RollingAverage avg = new RollingAverage(5, false);
    protected PIDController alignPID;
    private double speed = 1600;
    private double error = 25;

    /**
     * Manual override for the shooter.
     *
     * @param shooter the shooter subsystem
     * @param conveyor the conveyor subsystem
     * @param drivetrain drivetrain to align
     * @param limelight limelight to align
     */
    public midrangeShooter(Shooter shooter,
        Conveyor conveyor,
        Drivetrain drivetrain,
        Lemonlight limelight) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.alignPID = new PIDController(PIDValues.ALIGN_P, PIDValues.ALIGN_I, PIDValues.ALIGN_D);

        // TODO - Set these, including the constants
        alignPID.setTolerance(2, 9999999);
        alignPID.setSetpoint(0);
        addRequirements(shooter, drivetrain);

    }

    @Override
    public void initialize() {
        shooter.stop();

        shooter.extendHood();
    }
    /**
     * aligns to target.
     *
     * @param drivetrain drivetrain to use
     * @param horizontalOffset angle offset by
     * @return is aligned
     */

    public boolean driveAndAlign(Drivetrain drivetrain, double horizontalOffset) {
    
        //sets if align target based on ball color
        // if (isAccurate) {
        //     alignPID.setSetpoint(0);
        // } else {
        //     //alignPID.setSetpoint(0);
        //     alignPID.setSetpoint(TARGET_WRONG_COLOR_MISS);
        // }
        
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
        double horizontalOffset = limelight.getHorizontalOffset();
        boolean aligned = driveAndAlign(drivetrain, horizontalOffset);

        if (Functions.isWithin(avg.getAverage(), speed, error) && aligned) {
            shooter.setState(Shooter.States.READY_TO_FIRE);
            System.out.println("rpm: " + shooter.getShooterRPM());
        } else {
            shooter.setState(Shooter.States.NOT_SHOOTING);
        }
        //shooter.setMotorVolts(shooter.calculateVoltageFromPid(1700));
        shooter.setMotorTargetSpeed(speed);
    }

    @Override
    public void end(final boolean interrupted) {
        shooter.stop();
        drivetrain.stop();
        shooter.setState(Shooter.States.NOT_SHOOTING);

    }

    @Override
    public boolean isFinished() {
        System.out.println("existssssss: " + conveyor.doesBallExist());
        return !conveyor.doesBallExist();
    }
}