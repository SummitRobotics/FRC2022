package frc.robot.devices;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utilities.RollingAverage;

/**
 * Class for using the lidar v4.
 */
public class LidarV4 implements Lidar, Sendable {

    @SuppressWarnings("CheckStyle")
    private final I2C portI2C;
    private int value;

    private final RollingAverage rollingAverage;

    private final Runnable proximityReader;
    private Notifier thread;
    private Object valueLock;
    private Object loopLock;
    private Timer loopTimer;
    private double measuredLoopTime;

    /**
     * constructor.
     *
     * @param id the i2c id of the lidarV4
     */
    public LidarV4(int id) {
        portI2C = new I2C(Port.kMXP, id);
        value = 0;
        measuredLoopTime = 100;

        rollingAverage = new RollingAverage(5, true);

        valueLock = new Object();
        loopLock = new Object();
        loopTimer = new Timer();
        proximityReader = new Runnable() {
            @Override
            public void run() {
                loopTimer.start();
                updateValue(readDistance());
                loopTimer.stop();
                updateLoopTime(loopTimer.get());
                loopTimer.reset();
            }
        };
        thread = new Notifier(proximityReader);
        thread.startPeriodic(0.02);

    }

    public LidarV4() {
        this(0x62);
    }

    /**
     * reads the current distance from the lidar if one is available.
     */
    private int readDistance() {
        byte[] status = new byte[1];

        // checks if there is a valid measurement
        portI2C.read(0x01, 1, status);

        if ((status[0] & 0x01) == 0) {
            byte[] low = new byte[1];
            byte[] high = new byte[1];

            // reads distance from lidar
            portI2C.read(0x10, 1, low);
            portI2C.read(0x11, 1, high);

            int out = high[0];

            // fixes java using signed bytes
            if (out < 0) {
                out = ((high[0] & 0b01111111) + 128);
            }

            out = (out << 8);
            int out2 = low[0];

            if (out2 < 0) {
                out2 = ((low[0] & 0b01111111) + 128);
            }

            out = out + out2;

            // tells lidar to take another measurement
            portI2C.write(0x00, 0x04);

            return out;
        }
        return -1;
    }

    /**
     * Gets the most recent distance.
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
     * Returns how long measurements took.
     *
     * @return The amount of time, in milliseconds, that it took to measure distance
     */
    @Override
    public double getLoopTimeMilliseconds() {
        synchronized (loopLock) {
            return measuredLoopTime * 1000;
        }
    }

    /**
     * Changes the i2c id of the v4.
     *
     * @param id the id to change to
     */
    public void changeId(int id) {
        // enables flash
        portI2C.write(0xEA, 0x11);

        // reads device id and saves it
        byte[] idBuffer = new byte[5];
        portI2C.read(0x16, 4, idBuffer);

        idBuffer[4] = (byte) id;
        byte[] writeBuffer = new byte[6];
        writeBuffer[0] = 0x16;

        System.arraycopy(idBuffer, 0, writeBuffer, 1, 5);

        // unlocks address writing
        portI2C.writeBulk(writeBuffer);

        // writes address
        portI2C.write(0x1B, 0x01);
    }

    private void updateValue(int value) {
        if (value != -1) {
            synchronized (valueLock) {
                this.value = value;
                rollingAverage.update(value);
            }
        }
    }

    private void updateLoopTime(double time) {
        synchronized (loopLock) {
            measuredLoopTime = time;
        }
    }

    /**
     * Gets the average distance.
     *
     * @return average distance in cm
     */
    @Override
    public int getAverageDistance() {
        synchronized (valueLock) {
            return (int) rollingAverage.getAverage();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("LidarV4");
        builder.addDoubleProperty("avgDistance", this::getAverageDistance, null);
        builder.addDoubleProperty("distance", this::getDistance, null);
    }
}
