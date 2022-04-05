package frc.robot.devices;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Testable;
import java.util.HashMap;

/**
 * Contains methods for interfacing with the PCM (Pneumatics Control Module).
*/
public class PCM implements Sendable, Testable {

    private PneumaticHub hub;
    private Drivetrain drivetrain;
    private Timer timer;

    private static final double
        FULL_PRESSURE = 100,
        PARTIAL_PRESSURE = 75;

    private boolean hasReachedFullPressure;
    private boolean hasReachedPartialPressure;

    /**
     * Contains methods for interfacing with the PCM (Pneumatics Control Module).
     *
     * @param module The module number to construct
     * @param drivetrain The drivetrain subsystem
     */
    public PCM(int module, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        hasReachedFullPressure = false;
        hasReachedPartialPressure = false;
        timer = new Timer();
        hub = new PneumaticHub(module);
    }

    /**
     * Checks whether or not a solenoid channel is valid.
     *
     * @param channel the channel to be checked
     * @return whether or not the solenoid at the given channel is valid
     */
    public boolean checkSolenoidChannel(int channel) {
        return hub.checkSolenoidChannel(channel);
    }

    public void close() {
        hub.close();
    }

    /** Disables the compressor. */
    public void disableCompressor() {
        hub.disableCompressor();
    }

    /**
     * Enables the compressor.
     *
     * @param minPressure When the pressure drops below this the compressor will turn on.
     * @param maxPressure When the pressure goes above this the compressor will stop.
     */
    public void enableCompressorAnalog(double minPressure, double maxPressure) {
        hub.enableCompressorAnalog(minPressure, maxPressure);
    }

    public void enableCompressorDigital() {
        hub.enableCompressorDigital();
    }

    public void enableCompressorHybrid(double minPressure, double maxPressure) {
        hub.enableCompressorHybrid(minPressure, maxPressure);
    }

    public double getAnalogVoltage(int channel) {
        return hub.getAnalogVoltage(channel);
    }

    /**
     * Checks if the compressor is valid.
     *
     * @return Whether the compressor is valid.
     */
    public boolean getCompressor() {
        return hub.getCompressor();
    }

    public double getCompressorCurrent() {
        return hub.getCompressorCurrent();
    }

    public double getPressure(int channel) {
        return hub.getPressure(channel);
    }

    public double getPressure() {
        return hub.getPressure(0);
    }

    /**
     * Gets solenoid values.
     *
     * @return the solenoids.
     */
    public int getSolenoids() {
        return hub.getSolenoids();
    }

    /**
     * Gets the module number (useful for initializing Solenoids and DoubleSolenoids).
     *
     * @return the module number.
     */
    public int getModuleNumber() {
        return hub.getModuleNumber();
    }

    public boolean getPressureSwitch() {
        return hub.getPressureSwitch();
    }

    @Override
    public String getTestName() {
        return "Pneumatics";
    }

    @Override
    public double getAllowedTimeSeconds() {
        return 20;
    }

    @Override
    public HashMap<String, Boolean> initCustomTests() {
        HashMap<String, Boolean> result = new HashMap<String, Boolean>();
        result.put("Compressor", false);
        result.put("Pressure", false);

        timer.reset();
        timer.start();

        return result;
    }

    @Override
    public HashMap<String, Boolean> runCustomTests() {
        HashMap<String, Boolean> result = new HashMap<String, Boolean>();
        boolean pressureStatus = false;

        if (hasReachedFullPressure) {
            if (hasReachedPartialPressure) {
                pressureStatus = true;
            } else {
                if (timer.hasElapsed(0.05)) {
                    drivetrain.toggleShift();
                    timer.reset();
                    if (getPressure() < PARTIAL_PRESSURE) {
                        hasReachedPartialPressure = true;
                    }
                }
            }

        } else if (getPressure() > FULL_PRESSURE) {
            hasReachedFullPressure = true;
        }

        result.put("Compressor", getCompressor() && getCompressorCurrent() > 0);
        result.put("Pressure", pressureStatus);
        
        return result;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("compressor current", this::getCompressorCurrent, null);
        builder.addDoubleProperty("current pressure", this::getPressure, null);
        builder.addBooleanProperty("pressure switch", this::getPressureSwitch, null);
        
    }
}
