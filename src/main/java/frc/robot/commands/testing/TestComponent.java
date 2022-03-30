package frc.robot.commands.testing;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.devices.ColorSensor;
import frc.robot.devices.Lemonlight;
import frc.robot.devices.Lidar;
import frc.robot.oi.drivers.ShuffleboardDriver;
import frc.robot.utilities.Testable;
import frc.robot.utilities.lists.Colors;
import java.util.HashMap;

/**
 * Tests a testable subsystem.
 */
public class TestComponent extends CommandBase {

    private CANSparkMax[] motors;
    private ColorSensor[] colorSensors;
    private HashMap<String, Lemonlight> limelights;
    private Lidar[] lidars;
    private RelativeEncoder[] encoders;
    private double[] startingPositions;
    private HashMap<String, Boolean> testStates;
    private Testable toTest;

    private Timer timer;
    
    /**
     * Tests a testable subsystem.
     *
     * @param toTest The testable object
     */
    public TestComponent(Testable toTest) {
        this.toTest = toTest;

        motors = toTest.getMotors();
        colorSensors = toTest.getColorSensors();
        limelights = toTest.getLimelights();
        lidars = toTest.getLidars();

        for (int i = 0; i < motors.length; i++) {
            testStates.put("Motor " + motors[i].getDeviceId(), false);
            encoders[i] = motors[i].getEncoder();
        }

        for (int i = 0; i < colorSensors.length; i++) {
            testStates.put("Color Sensor " + i, false);
        }

        for (int i = 0; i < lidars.length; i++) {
            testStates.put("Lidar " + i, false);
        }

        for (HashMap.Entry<String, Lemonlight> set : limelights.entrySet()) {
            testStates.put(set.getKey(), false);
        }

        timer = new Timer();

        try {
            addRequirements((Subsystem) toTest);
        } catch (RuntimeException runtimeException) {
            System.out.println(
                "No requirements were added because the Testable object was not a subsystem.");
        }
    }

    @Override
    public void initialize() {
        for (int i = 0; i < motors.length; i++) {
            startingPositions[i] = encoders[i].getPosition();
            motors[i].set(toTest.getMotorTestSpeed());
        }

        for (HashMap.Entry<String, Boolean> set : toTest.initCustomTests().entrySet()) {
            testStates.put(set.getKey(), false);
        }

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        for (int i = 0; i < motors.length; i++) {
            if (encoders[i].getPosition() > startingPositions[i] + toTest.getMotorTestRotations()) {
                motors[i].set(0);
                testStates.replace("Motor " + motors[i].getDeviceId(), true);
            }
        }

        for (int i = 0; i < colorSensors.length; i++) {
            if (colorSensors[i].getLoopTimeMilliseconds() < toTest.getMaxSensorLoopMilliseconds()
                && colorSensors[i].getProximity() > 0
                && colorSensors[i].getColorString() != "Unknown") {
                testStates.replace("Color Sensor " + i, true);
            } else {
                testStates.replace("Color Sensor " + i, false);
            }
        }

        for (int i = 0; i < lidars.length; i++) {
            if (lidars[i].getLoopTimeMilliseconds() < toTest.getMaxSensorLoopMilliseconds()
                && lidars[i].getDistance() > 0) {
                testStates.replace("Lidar " + i, true);
            } else {
                testStates.replace("Lidar " + i, false);
            }
        }

        for (HashMap.Entry<String, Lemonlight> set : limelights.entrySet()) {
            NetworkTable networkTable =
                NetworkTableInstance.getDefault().getTable(set.getKey());
            if (set.getValue() != null
                && networkTable.getEntry("tv").getDouble(-1) != -1
                && networkTable.getEntry("tx").getDouble(10000) != 10000
                && networkTable.getEntry("ty").getDouble(10000) != 10000
                && networkTable.getEntry("ta").getDouble(10000) != 10000) {

                testStates.replace(set.getKey(), true);
            } else {
                testStates.replace(set.getKey(), false);
            }
        }

        testStates.putAll(toTest.runCustomTests());
    }

    @Override
    public void end(final boolean interrupted) {
        timer.stop();

        for (CANSparkMax motor : motors) {
            motor.set(0);
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
            String toPrint = toTest.getTestName() + ":";

            for (HashMap.Entry<String, Boolean> set : testStates.entrySet()) {
                if (!set.getValue()) {
                    toPrint += (" " + set.getKey() + ",");
                }
            }

            toPrint = toPrint.substring(0, toPrint.length() - 1);

            ShuffleboardDriver.statusDisplay.addStatus(
                "Test Status",
                "Test failed due to timeout | " + toPrint,
                Colors.RED,
                5
            );

            throw new RuntimeException("Test failed due to timeout: " + toPrint);

        } else {
            return false;
        }
    }
}
