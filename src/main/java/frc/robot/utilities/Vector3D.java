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
     *
     * @param other The other vector
     * @return The angle between the two vectors in Radians
     */
    public double getAngleRadians(Vector3D other) {
        return Math.acos((this.dotProduct(other)) / (this.getMagnitude() * other.getMagnitude()));
    }

    /**
     * Gets the angle between the current vector and the other vector.
     *
     * @param other The other vector
     * @return The angle between the two vectors in Degrees
     */
    public double getAngleDegrees(Vector3D other) {
        return Math.toDegrees(getAngleRadians(other));
    }
}
