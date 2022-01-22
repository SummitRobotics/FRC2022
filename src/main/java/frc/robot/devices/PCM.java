package frc.robot.devices;

import edu.wpi.first.wpilibj.PneumaticHub;

// TODO - Find better documentation and figure out what more of these methods do
/** Contains methods for interfacing with the PCM (Pneumatics Control Module) */
public class PCM {

  private PneumaticHub hub;

  public PCM() {
    hub = new PneumaticHub();
  }

  /**
   * Checks whether or not a solenoid channel is valid
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

  /** Disables the compressor */
  public void disableCompressor() {
    hub.disableCompressor();
  }

  /**
   * Enables the compressor
   *
   * @param minPressure
   * @param MaxPressure
   */
  public void enableCompressorAnalog(double minPressure, double MaxPressure) {
    hub.enableCompressorAnalog(minPressure, MaxPressure);
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
   * Checks if the compressor is valid
   *
   * @return
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

  /**
   * Gets solenoid values
   *
   * @return
   */
  public int getSolenoids() {
    return hub.getSolenoids();
  }

  /**
   * Gets the module number (useful for initializing Solenoids and DoubleSolenoids)
   *
   * @return
   */
  public int getModuleNumber() {
    return hub.getModuleNumber();
  }
}
