package frc.robot.devices;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * Contains methods for interfacing with the PDP (Power Distribution Panel).
 */
public class PDP implements Sendable {

    private PowerDistribution panel;

    public PDP(int id) {
        panel = new PowerDistribution(id, ModuleType.kRev);
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PDP");
        builder.addDoubleProperty("temperature", this::getTemperature, null);
        builder.addDoubleProperty("totalCurrent", this::getTotalCurrent, null);
        builder.addDoubleProperty("voltage", this::getVoltage, null);

    }
}
