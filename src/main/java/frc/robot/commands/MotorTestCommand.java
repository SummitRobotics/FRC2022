package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.lists.Colors;

/**
 * Command to automatically test motor functionality.
 */
public class MotorTestCommand extends CommandBase {

    private Shooter shooter;
    private Intake intake;

    private boolean shooterIsGood;
    private boolean intakeIsGood;
    private double shooterEncoderPos;
    private double intakeEncoderPos;

    private Timer timer;

    /**
     * Command to automatically test motor functionality.
     *
     * @param shooter The shooter subsystem
     * @param intake The intake subsystem
     */
    public MotorTestCommand(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        intakeIsGood = false;
        shooterIsGood = false;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        shooter.setMotorPower(0.3);
        intake.setIntakeMotorPower(0.3);
        shooterEncoderPos = shooter.getEncoderValue();
        intakeEncoderPos = intake.getIntakeEncoderPosition();
        timer.start();
    }

    @Override
    public void execute() {
        if (shooter.getEncoderValue() > shooterEncoderPos + 0.5) {
            shooterIsGood = true;
        }

        if (intake.getIntakeEncoderPosition() > intakeEncoderPos + 0.5) {
            intakeIsGood = true;
        }
    }

    @Override
    public void end(final boolean interrupted) {
        timer.stop();
        shooter.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(5)) {
            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test failed due to timeout",
                Colors.RED,
                5
            );
            throw new RuntimeException("Test failed due to timeout");

        } else if (shooterIsGood && intakeIsGood) {
            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test was successful",
                Colors.GREEN,
                5
            );
            return true;

        } else {
            return false;
        }
    }
}
