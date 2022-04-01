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
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Tests a testable subsystem.
 */
public class TestComponent extends CommandBase {

    private ArrayList<CANSparkMax> motors;
    private ArrayList<ColorSensor> colorSensors;
    private HashMap<String, Lemonlight> limelights;
    private ArrayList<Lidar> lidars;
    private ArrayList<RelativeEncoder> encoders;
    private ArrayList<Double> startingPositions;
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

        encoders = new ArrayList<RelativeEncoder>();
        startingPositions = new ArrayList<Double>();
        testStates = new HashMap<String, Boolean>();
        motors = toTest.getMotors();
        colorSensors = toTest.getColorSensors();
        limelights = toTest.getLimelights();
        lidars = toTest.getLidars();

        for (int i = 0; i < motors.size(); i++) {
            testStates.put("Motor " + motors.get(i).getDeviceId(), false);
            encoders.add(i, motors.get(i).getEncoder());
        }

        for (int i = 0; i < colorSensors.size(); i++) {
            testStates.put("Color Sensor " + i, false);
        }

        for (int i = 0; i < lidars.size(); i++) {
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
        for (int i = 0; i < motors.size(); i++) {
            startingPositions.add(i, encoders.get(i).getPosition());
            motors.get(i).set(toTest.getMotorTestSpeed());
        }

        for (HashMap.Entry<String, Boolean> set : toTest.initCustomTests().entrySet()) {
            testStates.put(set.getKey(), false);
        }

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        for (int i = 0; i < motors.size(); i++) {
            if (encoders.get(i).getPosition() > startingPositions.get(i) + toTest.getMotorTestRotations()) {
                motors.get(i).set(0);
                testStates.replace("Motor " + motors.get(i).getDeviceId(), true);
            }
        }

        for (int i = 0; i < colorSensors.size(); i++) {
            if (colorSensors.get(i).getLoopTimeMilliseconds() < toTest.getMaxSensorLoopMilliseconds()
                && colorSensors.get(i).getProximity() > 0
                && colorSensors.get(i).getColorString() != "Unknown") {
                testStates.replace("Color Sensor " + i, true);
            } else {
                testStates.replace("Color Sensor " + i, false);
            }
        }

        for (int i = 0; i < lidars.size(); i++) {
            if (lidars.get(i).getLoopTimeMilliseconds() < toTest.getMaxSensorLoopMilliseconds()
                && lidars.get(i).getDistance() > 0) {
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
