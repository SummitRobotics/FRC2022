package frc.robot.devices;

import edu.wpi.first.wpilibj.PneumaticHub;

public class PCM {

    private PneumaticHub hub;

    public PCM() {
        hub = new PneumaticHub();
    }

    public void close() {
        hub.close();
    }

    public void disableCompressor() {
        hub.disableCompressor();
    }

}
