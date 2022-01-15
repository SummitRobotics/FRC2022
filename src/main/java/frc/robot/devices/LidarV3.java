package frc.robot.devices;

import java.nio.ByteBuffer;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utilities.RollingAverage;;

/**
 * Device to manage the Lidar3
 */
public class LidarV3 implements Lidar {

    private RollingAverage rollingAverage;
    
    private boolean hasStartedMeasuring = false;

    public LidarV3() {

        m_port = (byte) Port.kOnboard.value;
		I2CJNI.i2CInitialize(m_port);

        rollingAverage = new RollingAverage(50, true);

        hasStartedMeasuring = false;

        startMeasuring();
    }
    
    private static final byte k_deviceAddress = 0x62;

	private final byte m_port;

	private final ByteBuffer m_buffer = ByteBuffer.allocateDirect(2);

	/**
	 * Tells the lidar to start taking measurements.
	 * Must be called before geting measurements
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
        if(!hasStartedMeasuring){
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

	// scarey
	private int writeRegister(int address, int value) {
		m_buffer.put(0, (byte) address);
		m_buffer.put(1, (byte) value);

		return I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 2);
	}

	// i dont understand how this works
	private short readShort(int address) {
		m_buffer.put(0, (byte) address);
		I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 1);
		I2CJNI.i2CRead(m_port, k_deviceAddress, m_buffer, (byte) 2);
        return m_buffer.getShort(0);
    }
}