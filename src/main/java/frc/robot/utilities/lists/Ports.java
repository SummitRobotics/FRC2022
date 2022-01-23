// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.lists;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Ports {

    public static final int
            // OI
            XBOX_PORT = 0,
            LAUNCHPAD_PORT = 1,
            JOYSTICK_PORT = 2,

            // drivetrain
            // TODO: Set ports
            LEFT_DRIVE_1 = 0,
            LEFT_DRIVE_2 = 0,
            LEFT_DRIVE_3 = 0,
            RIGHT_DRIVE_1 = 0,
            RIGHT_DRIVE_2 = 0,
            RIGHT_DRIVE_3 = 0,
            SHIFT_SOLENOID_UP = 0,
            SHIFT_SOLENOID_DOWN = 0,

            // LEDs
            LED_PORT = 0,
            LED_LENGTH = 0,

            // pneumatics
            PRESSURE_SENSOR = 0,
            PCM_1 = 0;
}
