package frc.robot.utilities.lists;

/**
 * PID values that we can repeat across more than one controller.
 */
public final class PIDValues {
    // TODO - configure
    public static final double
        // move PID
        MOVE_P = 0.01,
        MOVE_I = 0.0,
        MOVE_D = 0,

        // align PID
        ALIGN_P = 0.015,
        ALIGN_I = 0.02,
        ALIGN_D = 0.00,

        // climb PID
        CLIMB_P = 0.15,
        CLIMB_I = 0.1,
        CLIMB_D = 0;
}
