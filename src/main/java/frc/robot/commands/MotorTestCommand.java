package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.lists.Colors;

/**
 * Command to automatically test motor functionality.
 */
public class MotorTestCommand extends CommandBase {

    private Shooter shooter;
    private Intake intake;
    private Drivetrain drivetrain;
    private Conveyor conveyor;
    private Climb climb;

    private boolean shooterIsGood;
    private boolean intakeIsGood;
    private boolean leftDrivetrainIsGood;
    private boolean rightDrivetrainIsGood;
    private boolean indexConveyorIsGood;
    private boolean beltConveyorIsGood;
    private boolean leftClimbIsGood;
    private boolean rightClimbIsGood;

    private double shooterEncoderPos;
    private double intakeEncoderPos;
    private double leftDrivetrainEncoderPos;
    private double rightDrivetrainEncoderPos;
    private double indexConveyorEncoderPos;
    private double beltConveyorEncoderPos;
    private double leftClimbEncoderPos;
    private double rightClimbEncoderPos;

    private Timer timer;

    /**
     * Command to automatically test motor functionality.
     *
     * @param shooter The shooter subsystem
     * @param intake The intake subsystem
     * @param drivetrain The drivetrain subsystem
     * @param conveyor The conveyor subsystem
     * @param climb The climb subsystem
     */
    public MotorTestCommand(
        Shooter shooter,
        Intake intake,
        Drivetrain drivetrain,
        Conveyor conveyor,
        Climb climb) {

        this.shooter = shooter;
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.conveyor = conveyor;
        this.climb = climb;

        intakeIsGood = false;
        shooterIsGood = false;
        leftDrivetrainIsGood = false;
        rightDrivetrainIsGood = false;
        indexConveyorIsGood = false;
        beltConveyorIsGood = false;
        leftClimbIsGood = false;
        rightClimbIsGood = false;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        shooterEncoderPos = shooter.getEncoderValue();
        intakeEncoderPos = intake.getIntakeEncoderPosition();
        leftDrivetrainEncoderPos = drivetrain.getLeftEncoderPosition();
        rightDrivetrainEncoderPos = drivetrain.getRightEncoderPosition();
        indexConveyorEncoderPos = conveyor.getIndexEncoderPosition();
        beltConveyorEncoderPos = conveyor.getBeltEncoderPosition();
        leftClimbEncoderPos = climb.getLeftEncoderValue();
        rightClimbEncoderPos = climb.getRightEncoderValue();

        // TODO - adjust these speeds
        shooter.setMotorPower(0.3);
        intake.setIntakeMotorPower(0.3);
        drivetrain.setBothMotorPower(0.3);
        conveyor.setIndexMotorPower(0.3);
        conveyor.setBeltMotorPower(0.3);
        climb.setMotorPower(0.3);
        timer.start();
    }

    @Override
    public void execute() {
        if (shooter.getEncoderValue() > shooterEncoderPos + 0.5) {
            shooter.stop();
            shooterIsGood = true;
        }

        if (intake.getIntakeEncoderPosition() > intakeEncoderPos + 0.5) {
            intake.stop();
            intakeIsGood = true;
        }

        if (drivetrain.getLeftEncoderPosition() > leftDrivetrainEncoderPos + 0.5) {
            drivetrain.setLeftMotorPower(0);
            leftDrivetrainIsGood = true;
        }

        if (drivetrain.getRightEncoderPosition() > rightDrivetrainEncoderPos + 0.5) {
            drivetrain.setRightMotorPower(0);
            rightDrivetrainIsGood = true;
        }

        if (conveyor.getIndexEncoderPosition() > indexConveyorEncoderPos + 0.5) {
            conveyor.setIndexMotorPower(0);
            indexConveyorIsGood = true;
        }

        if (conveyor.getBeltEncoderPosition() > beltConveyorEncoderPos + 0.5) {
            conveyor.setBeltMotorPower(0);
            beltConveyorIsGood = true;
        }

        if (climb.getLeftEncoderValue() > leftClimbEncoderPos + 0.5) {
            climb.setLeftMotorPower(0);
            leftClimbIsGood = true;
        }

        if (climb.getRightEncoderValue() > rightClimbEncoderPos + 0.5) {
            climb.setRightMotorPower(0);
            rightClimbIsGood = true;
        }

    }

    @Override
    public void end(final boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(5)) {
            
            String whatWentWrong;
            
            if (!intakeIsGood) {
                whatWentWrong = "Intake";
            } else if (!shooterIsGood) {
                whatWentWrong = "Shooter";
            } else if (!leftDrivetrainIsGood) {
                whatWentWrong = "Left Drivetrain";
            } else if (!rightDrivetrainIsGood) {
                whatWentWrong = "Right Drivetrain";
            } else if (!indexConveyorIsGood) {
                whatWentWrong = "Conveyor Index";
            } else if (!beltConveyorIsGood) {
                whatWentWrong = "Conveyor Belt";
            } else if (!leftClimbIsGood) {
                whatWentWrong = "Left Climb";
            } else {
                whatWentWrong = "Right Climb";
            }
            
            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test failed due to timeout | " + whatWentWrong,
                Colors.RED,
                5
            );

            throw new RuntimeException("Test failed due to timeout: " + whatWentWrong);

        } else if (
                shooterIsGood
                && intakeIsGood
                && leftDrivetrainIsGood && rightDrivetrainIsGood
                && indexConveyorIsGood && beltConveyorIsGood
                && leftClimbIsGood && rightClimbIsGood) {

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
