package frc.robot.devices;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticHub;

// TODO - Find better documentation and figure out what more of these methods do
/** Contains methods for interfacing with the PCM (Pneumatics Control Module). */
public class PCM implements Sendable {

    private PneumaticHub hub;

    public PCM(int module) {
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
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("compressor current", this::getCompressorCurrent, null);
        builder.addDoubleProperty("current pressure", this::getPressure, null);
        builder.addBooleanProperty("pressure switch", this::getPressureSwitch, null);
        
    }
}
