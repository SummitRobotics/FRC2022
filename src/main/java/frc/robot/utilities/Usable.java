package frc.robot.utilities;

/**
 * interface for making a button or axis usable for MOes.
 */
public interface Usable {
    void using(Object user);

    void release(Object user);

    boolean inUse();
}
