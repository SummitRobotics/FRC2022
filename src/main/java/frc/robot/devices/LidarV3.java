package frc.robot.devices;

import java.nio.ByteBuffer;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utilities.RollingAverage;
;

/**
 * Device to manage the Lidar3
 */
public class LidarV3 implements Lidar {

    private RollingAverage rollingAverage;

    private boolean hasStartedMeasuring = false;

    public LidarV3() {

        PORT = (byte) Port.kOnboard.value;
        I2CJNI.i2CInitialize(PORT);

        rollingAverage = new RollingAverage(50, true);

        hasStartedMeasuring = false;

        startMeasuring();
    }

    private static final byte DEVICE_ADDRESS = 0x62;

    private final byte PORT;

    private final ByteBuffer BUFFER = ByteBuffer.allocateDirect(2);

    /**
     * Tells the lidar to start taking measurements. Must be called before geting measurements
     */
    public void startMeasuring() {
        writeRegister(0x04, 0x08 | 32); // default plus bit 5
        writeRegister(0x11, 0xff);
        writeRegister(0x00, 0x04);
    }

    /**
     * Tells the lidar to stop taking measurements
     */
    public void stopMeasuring() {
        writeRegister(0x11, 0x00);
    }

    /**
     * Gets the current distance measurement from the lidar
     *
     * @return the distance in cm
     */
    @Override
    public int getDistance() {
        if (!hasStartedMeasuring) {
            startMeasuring();
            hasStartedMeasuring = true;
        }
        short value = readShort(0x8f);
        rollingAverage.update(value);

        return value;
    }

    /**
     * Gets the average distance from the lidar
     *
     * @return the average distance in cm
     */
    @Override
    public int getAverageDistance() {
        getDistance();
        return (int) rollingAverage.getAverage();
    }

    // scary
    private int writeRegister(int address, int value) {
        BUFFER.put(0, (byte) address);
        BUFFER.put(1, (byte) value);

        return I2CJNI.i2CWrite(PORT, DEVICE_ADDRESS, BUFFER, (byte) 2);
    }

    // i dont understand how this works
    private short readShort(int address) {
        BUFFER.put(0, (byte) address);
        I2CJNI.i2CWrite(PORT, DEVICE_ADDRESS, BUFFER, (byte) 1);
        I2CJNI.i2CRead(PORT, DEVICE_ADDRESS, BUFFER, (byte) 2);
        return BUFFER.getShort(0);
    }
}
