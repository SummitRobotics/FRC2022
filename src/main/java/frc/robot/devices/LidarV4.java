package frc.robot.devices;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utilities.RollingAverage;

/**
 * Class for using the lidar v4.
 */
public class LidarV4 implements Lidar, Sendable {

    @SuppressWarnings("CheckStyle")
    private final I2C portI2C;
    private int value;

    private final RollingAverage rollingAverage;

    /**
     * constructor.
     *
     * @param id the i2c id of the lidarV4
     */
    public LidarV4(int id) {
        portI2C = new I2C(Port.kOnboard, id);
        value = 0;

        rollingAverage = new RollingAverage(10, true);
    }

    /**
     * reads the current distance from the lidar if one is available.
     */
    private void readDistance() {
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

            // prevent bad values
            if (out < 1000) {
                value = out;
            }
        }

        rollingAverage.update(value);
    }

    /**
     * Gets the most recent distance.
     *
     * @return the distance in cm
     */
    @Override
    public int getDistance() {
        readDistance();
        return value;
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

    /**
     * Gets the average distance.
     *
     * @return average distance in cm
     */
    @Override
    public int getAverageDistance() {
        readDistance();
        return (int) rollingAverage.getAverage();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("LidarV4");
        builder.addDoubleProperty("avgDistance", this::getAverageDistance, null);
        builder.addDoubleProperty("distance", this::getDistance, null);
    }
}
