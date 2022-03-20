package frc.robot.commands.testing;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.utilities.Testable;
import frc.robot.utilities.lists.Colors;

/**
 * Tests a testable subsystem.
 */
public class TestSubsystem extends CommandBase {

    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;
    private double[] startingPositions;
    private boolean[] areMotorsGood;
    private String[] motorNames;

    private Timer timer;
    
    /**
     * Tests a testable subsystem.
     *
     * @param toTest The testable object
     */
    public TestSubsystem(Testable toTest) {
        motors = toTest.getMotors();
        motorNames = toTest.getMotorNames();
        
        for (int i = 0; i < motors.length; i++) {
            areMotorsGood[i] = false;
            encoders[i] = motors[i].getEncoder();
        }

        timer = new Timer();

        addRequirements(toTest.getSubsystemObject());
    }

    @Override
    public void initialize() {
        for (int i = 0; i < motors.length; i++) {
            startingPositions[i] = encoders[i].getPosition();
            motors[i].set(0.3);
        }

        timer.start();
    }

    @Override
    public void execute() {
        for (int i = 0; i < motors.length; i++) {
            if (encoders[i].getPosition() > startingPositions[i] + 0.5) {
                motors[i].set(0);
                areMotorsGood[i] = true;
            }
        }
    }

    @Override
    public void end(final boolean interrupted) {
        timer.stop();

        for (int i = 0; i < motors.length; i++) {
            motors[i].set(0);
        }
    }

    @Override
    public boolean isFinished() {
        String whatIsWrong = "";

        for (int i = 0; i < motors.length; i++) {
            if (!areMotorsGood[i]) {
                whatIsWrong += (" " + motorNames[i]);
            }
        }

        if (whatIsWrong == "") {
            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test was successful",
                Colors.GREEN,
                5
            );
            return true;

        } else if (timer.hasElapsed(5)) {
            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test failed due to timeout |" + whatIsWrong,
                Colors.RED,
                5
            );
            throw new RuntimeException("Test failed due to timeout:" + whatIsWrong);

        } else {
            return false;
        }
    }
}
