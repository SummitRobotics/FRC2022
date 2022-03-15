package frc.robot.utilities.lists;

/**
 * PID values that we can repeat across more than one controller.
 */
public final class PIDValues {
    // TODO - configure
    public static final double
        // move PID
        MOVE_P = 0.04,
        MOVE_I = 0.02,
        MOVE_D = 0,

        // align PID
        ALIGN_P = 0.004,
        ALIGN_I = 0.004,
        ALIGN_D = 0,

        // climb PID
        CLIMB_P = 0.15,
        CLIMB_I = 0.1,
        CLIMB_D = 0;
}
