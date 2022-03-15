package frc.robot.devices;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.utilities.RollingAverage;
import java.nio.ByteBuffer;

/**
 * Device to manage the Lidar3.
 */
public class LidarV3 implements Lidar, Sendable {

    private final RollingAverage rollingAverage;

    private static final byte DEVICE_ADDRESS = 0x62;

    private final byte port;

    private final ByteBuffer byteBuffer = ByteBuffer.allocateDirect(2);

    private Object valueLock = new Object();
    private short value;
    private Notifier thread;

    private final Runnable proximityReader = new Runnable() {
        @Override
        public void run() {
            updateValue(readShort(DEVICE_ADDRESS));
        }
    };

    /**
     * Constructor to create a LidarV3.
     */
    public LidarV3() {

        port = (byte) Port.kOnboard.value;
        I2CJNI.i2CInitialize(port);

        rollingAverage = new RollingAverage(50, true);

        thread = new Notifier(proximityReader);
        startMeasuring();
    }

    /**
     * Tells the lidar to start taking measurements. Must be called before getting measurements
     */
    public void startMeasuring() {
        writeRegister(0x04, 0x08 | 32); // default plus bit 5
        writeRegister(0x11, 0xff);
        writeRegister(0x00, 0x04);
        thread.startPeriodic(0.02);
    }

    /**
     * Tells the lidar to stop taking measurements.
     */
    public void stopMeasuring() {
        writeRegister(0x11, 0x00);
        thread.stop();
    }

    /**
     * Gets the current distance measurement from the lidar.
     *
     * @return the distance in cm
     */
    @Override
    public int getDistance() {
        synchronized (valueLock) {
            return value;
        }
    }

    /**
     * Gets the average distance from the lidar.
     *
     * @return the average distance in cm
     */
    @Override
    public int getAverageDistance() {
        synchronized (valueLock) {
            return (int) rollingAverage.getAverage();
        }
    }

    private void updateValue(short value) {
        synchronized (valueLock) {
            this.value = value;
            rollingAverage.update(value);
        }
    }

    // scary
    private int writeRegister(int address, int value) {
        byteBuffer.put(0, (byte) address);
        byteBuffer.put(1, (byte) value);

        return I2CJNI.i2CWrite(port, DEVICE_ADDRESS, byteBuffer, (byte) 2);
    }

    // I don't understand how this works
    private short readShort(int address) {
        byteBuffer.put(0, (byte) address);
        I2CJNI.i2CWrite(port, DEVICE_ADDRESS, byteBuffer, (byte) 1);
        I2CJNI.i2CRead(port, DEVICE_ADDRESS, byteBuffer, (byte) 2);
        return byteBuffer.getShort(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("LidarV3");
        builder.addDoubleProperty("avgDistance", this::getAverageDistance, null);
        builder.addDoubleProperty("distance", this::getDistance, null);
    }
}
