package frc.robot.utilities;

import java.util.Vector;

/**
 * Utility class for 3D vectors.
 */
public class Vector3D {
    private double xComponent;
    private double yComponent;
    private double zComponent;

    /**
     * Creates a new 3D vector.
     *
     * @param xComponent The magnitude of the vector in the x or i-hat component
     * @param yComponent The magnitude of the vector in the y or j-hat component
     * @param zComponent The magnitude of the vector in the z or k-hat component
     */
    public Vector3D(double xComponent, double yComponent, double zComponent) {
        this.xComponent = xComponent;
        this.yComponent = yComponent;
        this.zComponent = zComponent;
    }

    /**
     * Enum for referencing different planes.
     */
    public enum Plane {
        XYPlane,
        XZPlane,
        YZPlane
    }

    public enum Axis{
        XAxis,
        YAxis,
        ZAxis
    }

    /**
     * Creates a vector with x, y, z components at 0.
     */
    public Vector3D() {
        this(0,0,0);
    }

    /**
     * Returns the x or i-hat component of the vector.
     *
     * @return x component
     */
    public double getXComponent() {
        return xComponent;
    }

    /**
     * Returns the y or j-hat component of the vector.
     *
     * @return y component
     */
    public double getYComponent() {
        return yComponent;
    }

    /**
     * Sets a new x for the vector.
     *
     * @param value The new value
     */
    public Vector3D setXComponent(double value) {
        xComponent = value;
        return this;
    }

    /**
     * Sets a new y for the vector.
     *
     * @param value The new value
     */
    public Vector3D setYComponent(double value) {
        yComponent = value;
        return this;
    }

    /**
     * Sets a new z for the vector.
     *
     * @param value The new value
     */
    public Vector3D setZComponent(double value) {
        zComponent = value;
        return this;
    }

    /**
     * Returns the z or k-hat component of the vector.
     *
     * @return z component
     */
    public double getZComponent() {
        return zComponent;
    }

    /**
     * Returns the magnitude of the vector.
     *
     * @return The magnitude of the vector
     */
    public double getMagnitude() {
        return Math.sqrt((xComponent * xComponent) + (yComponent * yComponent) + (zComponent * zComponent));
    }

    /**
     * Creates a copy of the current vector.
     *
     * @return Creates a copy of the curent vector
     */
    public Vector3D copy() {
        return new Vector3D(xComponent, yComponent, zComponent);
    }

    /**
     * Scales the current vector by a value.
     *
     * @param scalar The value to scale the vector by
     * @return The current object
     */
    public Vector3D scale(double scalar) {
        xComponent *= scalar;
        yComponent *= scalar;
        zComponent *= scalar;
        return this;
    }

    /**
     * Creates a new vector scaled up by a value.
     *
     * @param scalar The value to scale the new Vector by
     * @return A new scaled vector
     */
    public Vector3D scaleNew(double scalar) {
        return new Vector3D(xComponent * scalar, yComponent * scalar, zComponent * scalar);
    }

    /**
     * Normalizes the current vector by making the magnitude 1.
     *
     * @return The current object
     */
    public Vector3D normalize() {
        scale(1 / getMagnitude());
        return this;
    }

    /**
     * Creates a new Vector in the same direction as the current vector with a magnitude of 1.
     *
     * @return A new normalized vector.
     */
    public Vector3D normalizeNew() {
        return scaleNew(1 / getMagnitude());
    }

    /**
     * Adds the other vector to the current vector.
     *
     * @param other The other vector to add to
     * @return The current object
     */
    public Vector3D add(Vector3D other) {
        xComponent += other.getXComponent();
        yComponent += other.getYComponent();
        zComponent += other.getZComponent();
        return this;
    }

    /**
     * Adds the other vector to the current vector creating a new object.
     *
     * @param other The other vector to add to
     * @return A new vector whose components are the sum of the current vector and the other vector
     */
    public Vector3D addNew(Vector3D other) {
        return new Vector3D(
            xComponent + other.getXComponent(),
            yComponent + other.getYComponent(),
            zComponent + other.getZComponent()
        );
    }

    /**
     * Subtracts the other vector to the current vector.
     *
     * @param other The other vector to subtract to
     * @return The current object
     */
    public Vector3D subtract(Vector3D other) {
        xComponent -= other.getXComponent();
        yComponent -= other.getYComponent();
        zComponent -= other.getZComponent();
        return this;
    }

    /**
     * Subtracts the other vector to the current vector creating a new object.
     *
     * @param other The other vector to subtract to
     * @return A new vector whose components are the difference of the current vector and the other vector
     */
    public Vector3D subtractNew(Vector3D other) {
        return new Vector3D(
            xComponent - other.getXComponent(),
            yComponent - other.getYComponent(),
            zComponent - other.getZComponent()
        );
    }

    /**
     * Computes the dot product between two vectors.
     * This * Other
     *
     * @param other The other vector
     * @return The dot product between th two vectors
     */
    public double dotProduct(Vector3D other) {
        return ((xComponent * other.getXComponent()) + (yComponent * other.getYComponent()) + (zComponent * other.zComponent));
    }

    /**
     * Computes the cross product between two vectors creating a new Vector in the process.
     * This x Other
     *
     * @param other The other vector
     * @return A new vector made from the cross product of the current vector and the other vector.
     */
    public Vector3D crossProduct(Vector3D other) {
        return new Vector3D(
            yComponent * other.getZComponent() - zComponent * other.getYComponent(),
            zComponent * other.getXComponent() - xComponent * other.getZComponent(),
            xComponent * other.getYComponent() - yComponent * other.getXComponent()
        );
    }

    /**
     * Gets the angle between the current vector and the other vector.
     * Returns 0 if either vector is 0
     *
     * @param other The other vector
     * @return The angle between the two vectors in Radians
     */
    public double getAngleRadians(Vector3D other) {
        if (this.getMagnitude() * other.getMagnitude() == 0) {
            return 0;
        }
        return Math.acos((this.dotProduct(other)) / (this.getMagnitude() * other.getMagnitude()));
    }

    /**
     * Gets the angle between the current vector and the other vector.
     * Returns 0 if either vector is 0
     *
     * @param other The other vector
     * @return The angle between the two vectors in Degrees
     */
    public double getAngleDegrees(Vector3D other) {
        return Math.toDegrees(getAngleRadians(other));
    }

    /**
     * Gets the angle in a certain plane compared to an axis.
     *
     * @param plane The plane we are looking at
     * @param axis The axis we are comparing it to
     * @return The angle in that plane.
     */
    public double getAngleInPlane(Plane plane, Axis axis) {
        Vector3D tempVector = this.copy();
        Vector3D comparisonVector;

        switch (plane) {
            case XYPlane:
                tempVector.setZComponent(0);
                break;
            case XZPlane:
                tempVector.setYComponent(0);
                break;
            case YZPlane:
                tempVector.setXComponent(0);
                break;
            default:
                break;
        }

        switch (axis) {
            case XAxis:
                comparisonVector = new Vector3D(1, 0, 0);
                break;
            case YAxis:
                comparisonVector = new Vector3D(0, 1, 0);
                break;
            case ZAxis:
                comparisonVector = new Vector3D(0, 0, 1);
                break;
            default:
                comparisonVector = new Vector3D();
                break;
        }

        return tempVector.getAngleRadians(comparisonVector);
    }

    @Override
    public String toString() {
        return String.format("3D Vector - (X:%f, Y:%f, Z:%f)", xComponent, yComponent, zComponent);
    }
}
