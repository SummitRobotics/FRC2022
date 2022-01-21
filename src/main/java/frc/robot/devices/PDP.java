package frc.robot.devices;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PDP {

    private PowerDistribution panel;

    public PDP() {
        panel = new PowerDistribution();
    }

    public void clearStickyFaults() {
        panel.clearStickyFaults();
    }
    
    public void close() {
        panel.close();
    }

    public double getCurrent(int channel) {
        return panel.getCurrent(channel);
    }

    public boolean getSwitchableChannel() {
        return panel.getSwitchableChannel();
    }

    public double getTemperature() {
        return panel.getTemperature();
    }

    public double getTotalCurrent() {
        return panel.getTotalCurrent();
    }

    public double getTotalEnergy() {
        return panel.getTotalEnergy();
    }

    public double getTotalPower() {
        return panel.getTotalPower();
    }

    public double getVoltage() {
        return panel.getVoltage();
    }

    public void resetTotalEnergy() {
        panel.resetTotalEnergy();
    }

    public void toggleSwitchableChannel() {
        panel.setSwitchableChannel(!panel.getSwitchableChannel());
    }
}
