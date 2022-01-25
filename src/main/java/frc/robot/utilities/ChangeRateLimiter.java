/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.util.Vector;

/**
 * Rate limiter class.
 */
public class ChangeRateLimiter {

    private final double rate;
    private double old;

    public static Vector<ChangeRateLimiter> allLimiters = new Vector<>();

    /**
     * new rate limiter with starting value of 0.
     *
     * @param rate the maximum change each time the function is called
     */
    public ChangeRateLimiter(double rate) {
        this.rate = rate;
        old = 0;
        allLimiters.add(this);
    }

    /**
     * new rate limiter.
     *
     * @param rate          the maximum change each time the function is called
     * @param startingValue the value to start out to allow things to run at full at init
     */
    public ChangeRateLimiter(double rate, double startingValue) {
        this.rate = rate;
        old = startingValue;
        allLimiters.add(this);
    }

    /**
     * limits a rate.
     *
     * @param value the value that's changing
     * @return the rate-limited value
     */
    public double getRateLimitedValue(double value) {
        if (value > old + rate) {
            value = old + rate;
            old = value;
        } else if (value < old - rate) {
            value = old - rate;
            old = value;
        } else {
            old = value;
        }

        return value;
    }

    public void resetOld() {
        old = 0;
    }

    /**
     * Resets all rate limiters.
     */
    public static void resetAllChangeRateLimiters() {
        for (ChangeRateLimiter x : allLimiters) {
            x.resetOld();
        }
    }
}
