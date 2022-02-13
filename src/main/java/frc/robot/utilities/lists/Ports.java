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

            // Drivetrain
            // TODO: Set ports
            LEFT_DRIVE_1 = 10,
            LEFT_DRIVE_2 = 11,
            LEFT_DRIVE_3 = 12,
            RIGHT_DRIVE_1 = 13,
            RIGHT_DRIVE_2 = 14,
            RIGHT_DRIVE_3 = 15,
            SHIFT_SOLENOID_UP = 1,
            SHIFT_SOLENOID_DOWN = 2,

            // Shooter
            SHOOTER_MOTOR = 16,
            HOOD_SOLENOID = 3,

            // Climb
            LEFT_CLIMB_MOTOR = 17,
            RIGHT_CLIMB_MOTOR = 18,
            PIVOT_CLIMB_SOLENOID = 4,
            LEFT_DETACH_SOLENOID = 5,
            RIGHT_DETACH_SOLENOID = 6,

            // LEDs
            LED_PORT = 0,
            LED_LENGTH = 1000,

            // pneumatics
            PRESSURE_SENSOR = 0,
            PCM_1 = 1,

            PDP = 2, 

            // conveyor
            FRONT_CONVEYOR = 19,
            BACK_CONVEYOR = 20,

            // intake
            INTAKE_MOTOR = 21,
            INTAKE_SOLENOID = 7;
}
