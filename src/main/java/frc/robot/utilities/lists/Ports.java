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
            SHIFT_SOLENOID = 4,

            // Shooter
            SHOOTER_MOTOR_1 = 30,
            SHOOTER_MOTOR_2 = 31,
            HOOD_SOLENOID = 7,

            // Climb
            LEFT_CLIMB_MOTOR = 21,
            RIGHT_CLIMB_MOTOR = 20,
            PIVOT_CLIMB_SOLENOID = 6,
            LEFT_DETACH_SOLENOID = 2,
            RIGHT_DETACH_SOLENOID = 0,
            LEFT_LIMIT_SWITCH = 0,
            RIGHT_LIMIT_SWITCH = 1,
            // LEDs
            LED_PORT = 0,
            LED_LENGTH = 114,

            // pneumatics
            PRESSURE_SENSOR = 0,
            PCM_1 = 2,
            PDP = 1,

            // conveyor
            FRONT_CONVEYOR = 40,
            BACK_CONVEYOR = 41,

            // intake
            INTAKE_MOTOR = 50,
            INTAKE_SOLENOID = 3;
}
