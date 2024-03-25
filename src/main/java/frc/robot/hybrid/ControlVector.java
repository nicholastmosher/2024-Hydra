package frc.robot.hybrid;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N6;

/**
 * An encoding of all the dynamic control signals for the robot.
 *
 * Dynamic controls are ones with a range (i.e. 0.0 to 1.0) rather than a
 * binary button-like or toggle-like control.
 */
public class ControlVector extends Vector<N6> {
    /**
     * Create a new ControlVector whose values are all zero
     */
    public ControlVector() {
        this(0, 0, 0, 0, 0, 0);
    }

    /**
     * Create a new ControlVector with the given values
     */
    public ControlVector(
            double swerveFieldX,
            double swerveFieldY,
            double swerveRobotX,
            double swerveRobotY,
            double swerveRotation,
            double armPower
    ) {
        super(Nat.N6());
        this.set(
                swerveFieldX,
                swerveFieldY,
                swerveRobotX,
                swerveRobotY,
                swerveRotation,
                armPower
        );
    }

    /**
     * Create a new ControlVector that's a copy of the given ControlVector
     */
    public ControlVector(ControlVector toCopy) {
        this(
                toCopy.swerveFieldX(),
                toCopy.swerveFieldY(),
                toCopy.swerveRobotX(),
                toCopy.swerveRobotY(),
                toCopy.swerveRotation(),
                toCopy.armPower()
        );
    }

    /**
     * Create a new ControlVector using the given field-relative X, Y, and rotation
     * All other control values are set to 0.
     *
     * @param swerveFieldX field-relative X power
     * @param swerveFieldY field-relative Y power
     * @param swerveRotation rotation power
     */
    public static ControlVector fromFieldRelative(double swerveFieldX, double swerveFieldY, double swerveRotation) {
        return new ControlVector(swerveFieldX, swerveFieldY, 0.0, 0.0, swerveRotation, 0.0);
    }

    /**
     * Create a new ControlVector using the given robot-relative X, Y, and rotation
     * All other control values are set to 0.
     *
     * @param swerveRobotX robot-relative X power
     * @param swerveRobotY robot-relative Y power
     * @param swerveRotation rotation power
     */
    public static ControlVector fromRobotRelative(double swerveRobotX, double swerveRobotY, double swerveRotation) {
        return new ControlVector(0.0, 0.0, swerveRobotX, swerveRobotY, swerveRotation, 0.0);
    }

    /**
     * Sets the control values of this ControlVector, modifying in-place
     *
     * @param swerveFieldX Power to apply to field-X coordinate (-1.0 to 1.0)
     * @param swerveFieldY Power to apply to field-Y coordinate (-1.0 to 1.0)
     * @param swerveRobotX Power to apply to robot-X coordinate (-1.0 to 1.0)
     * @param swerveRobotY Power to apply to robot-Y coordinate (-1.0 to 1.0)
     * @param swerveRotation Power to apply to rotation (-1.0 to 1.0)
     * @param armPower Power to apply to arm movement (-1.0 to 1.0)
     */
    // Private because it has the same name as Matrix.set, which is confusing
    private void set(
            double swerveFieldX,
            double swerveFieldY,
            double swerveRobotX,
            double swerveRobotY,
            double swerveRotation,
            double armPower
    ) {
        this.set(0, 0, swerveFieldX);
        this.set(1, 0, swerveFieldY);
        this.set(2, 0, swerveRobotX);
        this.set(3, 0, swerveRobotY);
        this.set(4, 0, swerveRotation);
        this.set(5, 0, armPower);
    }

    /**
     * @return the control value for the Swerve drive X translation (field space)
     */
    public double swerveFieldX() {
        return this.get(0, 0);
    }

    /**
     * @return the control value for the Swerve drive Y translation (field space)
     */
    public double swerveFieldY() {
        return this.get(1, 0);
    }

    /**
     * @return the control value for the Swerve drive X translation (robot space)
     */
    public double swerveRobotX() {
        return this.get(2, 0);
    }

    /**
     * @return the control value for the Swerve drive Y translation (robot space)
     */
    public double swerveRobotY() {
        return this.get(3, 0);
    }

    /**
     * @return the control value for Swerve drive rotation (-1.0 to 1.0)
     */
    public double swerveRotation() {
        return this.get(4, 0);
    }

    /**
     * @return the control value for moving the arm (-1.0 to 1.0)
     */
    public double armPower() {
        return this.get(5, 0);
    }

    /**
     * Set the swerve field-X coordinate of this ControlVector
     */
    public ControlVector setSwerveFieldX(double fieldX) {
        this.set(0, 0, fieldX);
        return this;
    }

    /**
     * Set the swerve field-X coordinate of this ControlVector
     */
    public ControlVector setSwerveFieldY(double fieldY) {
        this.set(1, 0, fieldY);
        return this;
    }

    /**
     * Set the swerve robot-X coordinate of this ControlVector
     */
    public ControlVector setSwerveRobotX(double robotX) {
        this.set(2, 0, robotX);
        return this;
    }

    /**
     * Set the swerve robot-Y coordinate of this ControlVector
     */
    public ControlVector setSwerveRobotY(double robotY) {
        this.set(3, 0, robotY);
        return this;
    }

    /**
     * Set the swerve rotation coordinate of this ControlVector
     */
    public ControlVector setSwerveRotation(double rotation) {
        this.set(4, 0, rotation);
        return this;
    }

    /**
     * Set the swerve field-X coordinate of this ControlVector
     */
    public ControlVector setArmPower(double armPower) {
        this.set(5, 0, armPower);
        return this;
    }

    /**
     * Add the values of this vector to the corresponding values of the other vector.
     * This ControlVector remains unmodified.
     *
     * @return a new ControlVector with the calculated values
     */
    public ControlVector plus(ControlVector other) {
        return new ControlVector(
                this.swerveFieldX() + other.swerveFieldX(),
                this.swerveFieldY() + other.swerveFieldY(),
                this.swerveRobotX() + other.swerveRobotX(),
                this.swerveRobotY() + other.swerveRobotY(),
                this.swerveRotation() + other.swerveRotation(),
                this.armPower() + other.armPower()
        );
    }

    /**
     * Multiply the values of this vector by a given scalar value
     * This ControlVector remains unmodified.
     *
     * @return a new ControlVector with the calculated values
     */
    public ControlVector times(double scalar) {
        return new ControlVector(
                this.swerveFieldX() * scalar,
                this.swerveFieldY() * scalar,
                this.swerveRobotX() * scalar,
                this.swerveRobotY() * scalar,
                this.swerveRotation() * scalar,
                this.armPower() * scalar
        );
    }

    /**
     * Calculate a ChassisSpeeds from both the field-relative controls and robot-relative controls
     *
     * @param heading the current heading of the robot
     */
    public ChassisSpeeds calculateChassisSpeeds(Rotation2d heading) {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveFieldX(),
                swerveFieldY(),
                swerveRotation(),
                heading
        );

        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(
                swerveFieldX(),
                swerveFieldY(),
                swerveRotation()
        );

        return new ChassisSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond + robotRelativeSpeeds.vxMetersPerSecond,
                fieldRelativeSpeeds.vyMetersPerSecond + robotRelativeSpeeds.vyMetersPerSecond,
                fieldRelativeSpeeds.omegaRadiansPerSecond + robotRelativeSpeeds.omegaRadiansPerSecond
        );
    }
}
