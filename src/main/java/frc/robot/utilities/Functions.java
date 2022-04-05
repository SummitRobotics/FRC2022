package frc.robot.utilities;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.FollowTrajectoryThreaded;
import frc.robot.subsystems.Drivetrain;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;

/**
 * Contains various static utility functions for use throughout the program.
 */
public class Functions {

    /**
     * Clamps a double between two values.
     *
     * @param in  the input value to clamp
     * @param max the maximum you want it to be
     * @param min the minimum for it to be
     * @return the clamped double
     */
    public static double clampDouble(double in, double max, double min) {
        if (in > max) {
            return max;
        } else {
            return Math.max(in, min);
        }
    }

    /**
     * returns input value with deadzone applied.
     *
     * @param deadRange the range in both directions to be dead
     * @param in        the input value to kill
     * @return the value to be used
     */
    public static double deadzone(double deadRange, double in) {
        if (Math.abs(in) < deadRange) {
            return 0;
        } else {
            return in;
        }
    }

    /**
     * Tells if value is within a target range.
     *
     * @param toCompare the value to compare
     * @param target    the target value
     * @param error     the valid range around the target
     * @return if the value is within the range
     */
    public static boolean isWithin(double toCompare, double target, double error) {
        return Math.abs(toCompare - target) <= (error / 2);
    }

    /**
     * returns true if the abs of A is bigger than B.
     *
     * @param inputA reference value
     * @param inputB comparing value
     * @return true if abs of reference is bigger
     */
    public static boolean absGreater(double inputA, double inputB) {
        return Math.abs(inputA) > Math.abs(inputB);
    }

    /**
     * saves an object to a file.
     *
     * @param <T>    the object type
     * @param object the object to save
     * @param path   the path, including the file name, to save
     * @apiNote WARNING,this can fail and not save the object!
     */
    public static <T> void saveObjectToFile(T object, String path) {
        try {
            FileOutputStream fileOut = new FileOutputStream(path);
            ObjectOutputStream objectOut = new ObjectOutputStream(fileOut);
            objectOut.writeObject(object);
            objectOut.close();
            fileOut.close();

        } catch (Exception ex) {
            ex.printStackTrace();
            throw (new RuntimeException("saving failed"));
        }
    }

    /**
     * reads an object out of a file.
     *
     * @param <T>  the object type
     * @param path the path where the object is
     * @return the object read from the file
     * @throws Exception throws an exception if the fie can not be read
     */
    @SuppressWarnings("unchecked")
    public static <T> T retrieveObjectFromFile(String path) throws Exception {
        FileInputStream fis = new FileInputStream(path);
        ObjectInputStream ois = new ObjectInputStream(fis);
        T result = (T) ois.readObject();
        ois.close();
        return result;
    }

    /**
     * creates a trajectory command from a file name.
     *
     * @param drivetrain the drivetrain subsystem
     * @param fileName   the path to and name of the file
     * @return the FollowTrajectoryThreaded for the path file
     * @throws IOException thrown if the file was not found or readable
     */
    public static Command splineCommandFromFile(Drivetrain drivetrain, String fileName)
            throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(fileName);

        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);

        return new FollowTrajectoryThreaded(drivetrain, trajectory);
    }
    /**
     * takes a list of numbers and gets the median value of them. 
     *
     * @param toBeAveraged Arraylist of numbers to get the median of
     * @return the median value
     */

    public static double medianWithoutExtraneous(ArrayList<Double> toBeAveraged) {
        Collections.sort(toBeAveraged);
        double mean = 0;
        for (double i : toBeAveraged) {
            mean += i;
        }
        mean /= 3;
        final double asfasd = mean;

        toBeAveraged.removeIf(e -> (e != 0 && Math.abs((asfasd - e) / e) > .2));
        if (toBeAveraged.size() == 0) return 0;
        return toBeAveraged.get((int) (toBeAveraged.size() / 2));
    }

    /**
     * finds the point closest to the input value. if 2 points are the same dist it returns the first in the points array.
     *
     * @param value the value to find the point closest to
     * @param points the points that the value should be quantised to
     * @return the point closest to the input value
     */
    public static double findClosestPoint(double value, double[] points) {
        double out = Double.NaN;
        double minError = Double.POSITIVE_INFINITY;
        for (double x : points) {
            double error = Math.abs(x - value);
            if (error < minError) {
                minError = error;
                out = x;
            }
        }

        return out;
    }

    public static double findClosestPoint(double value, Double[] points) {
        double out = Double.NaN;
        double minError = Double.POSITIVE_INFINITY;
        for (double x : points) {
            double error = Math.abs(x - value);
            if (error < minError) {
                minError = error;
                out = x;
            }
        }

        return out;
    }
}
