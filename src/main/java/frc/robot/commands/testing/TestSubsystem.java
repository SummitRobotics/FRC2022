package frc.robot.commands.testing;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.utilities.Testable;
import frc.robot.utilities.lists.Colors;
import java.util.HashMap;

/**
 * Tests a testable subsystem.
 */
public class TestSubsystem extends CommandBase {

    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;
    private double[] startingPositions;
    private HashMap<String, Boolean> testStates;
    private Testable toTest;
    private Subsystem subsystem;
    
    private Timer timer;
    
    /**
     * Tests a testable subsystem.
     *
     * @param toTest The testable object
     */
    public TestSubsystem(Testable toTest) {
        this.toTest = toTest;
        motors = toTest.getMotors();
        subsystem = toTest.getSubsystemObject();
        
        for (int i = 0; i < motors.length; i++) {
            testStates.put(Integer.toString(motors[i].getDeviceId()), false);
            encoders[i] = motors[i].getEncoder();
        }

        timer = new Timer();

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < motors.length; i++) {
            startingPositions[i] = encoders[i].getPosition();
            motors[i].set(toTest.getMotorTestSpeed());
        }

        timer.start();
    }

    @Override
    public void execute() {
        for (int i = 0; i < motors.length; i++) {
            if (encoders[i].getPosition() > startingPositions[i] + toTest.getMotorTestRotations()) {
                motors[i].set(0);
                testStates.replace(Integer.toString(motors[i].getDeviceId()), true);
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
        boolean isSuccessful = true;

        for (HashMap.Entry<String, Boolean> set : testStates.entrySet()) {
            isSuccessful &= set.getValue();
        }

        if (isSuccessful) {
            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test was successful",
                Colors.GREEN,
                5
            );
            return true;

        } else if (timer.hasElapsed(toTest.getAllowedTimeSeconds())) {
            String toPrint = "";

            for (HashMap.Entry<String, Boolean> set : testStates.entrySet()) {
                if (!set.getValue()) {
                    toPrint += (" " + subsystem.getClass().getCanonicalName()
                        + ": " + set.getKey() + ",");
                }
            }

            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test failed due to timeout |" + toPrint,
                Colors.RED,
                5
            );
            throw new RuntimeException("Test failed due to timeout:" + toPrint);

        } else {
            return false;
        }
    }
}
