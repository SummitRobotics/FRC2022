package frc.robot.utilities;

/**
 * interface for making a button or axis usable for MOes
 */
public interface Usable {
	public void using(Object user);
	public void release(Object user);
	public boolean inUse();
}