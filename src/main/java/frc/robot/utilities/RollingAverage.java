package frc.robot.utilities;

import java.util.Arrays;

/**
 * Gets a running average of a value
 */
public class RollingAverage {

    private Double[] rollingAverageArray;
    private int rollingTarget;
    private boolean initialized;
    private boolean fill;

    /**
     * creates new rollingavrage
     * @param size the number of values you want to avrage
     * @param fill true will fill the array with the first value input on the first update, false will fill the array with 0's
     */
    public RollingAverage(int size, boolean fill) {
        rollingAverageArray = new Double[size];

        initialized = false;
        this.fill = fill;
    }

    public void set(double value) {
        Arrays.fill(rollingAverageArray, value);
    }

    public void reset() {
        set(0);
        initialized = false;
    }

    /**
     * Adds a new value to the average
     * 
     * @param value the new value
     */
    public void update(double value) {
        if (!initialized) {
            set(fill? value: 0.0);

            initialized = true;
            
        } else {
            rollingAverageArray[rollingTarget] = value;
        }

        rollingTarget = (rollingTarget + 1) % rollingAverageArray.length;
    }

    /**
     * Gets the running average
     * 
     * @return the average
     */
    public double getAverage() {
        double value = 0;
        for (Double d : rollingAverageArray) {
            value += d;
        }

        value /= rollingAverageArray.length;
        return value;
    }
}