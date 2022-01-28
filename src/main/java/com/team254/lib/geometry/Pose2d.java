package com.team254.lib.geometry;

import com.team254.lib.util.Util;

/**
 * Represents a 2d pose (rigid transform) containing translational and rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Pose2d implements IPose2d<Pose2d> {
    protected static final Pose2d kIdentity = new Pose2d();

    public static Pose2d identity() {
        return kIdentity;
    }

    private final static double kEps = 1E-9;

    protected final Translation2d translation_;
    protected final Rotation2d rotation_;
    protected final Rotation2d chassisHeading_;

    public Pose2d() {
        translation_ = new Translation2d();
        rotation_ = new Rotation2d();
        chassisHeading_ = new Rotation2d();
    }

    public Pose2d(double x, double y, final Rotation2d rotation) {
        translation_ = new Translation2d(x, y);
        rotation_ = rotation;
        chassisHeading_ = Rotation2d.identity();
    }

    public Pose2d(double x, double y, final Rotation2d rotation, Rotation2d chassisHeading) {
        translation_ = new Translation2d(x, y);
        rotation_ = rotation;
        chassisHeading_ = chassisHeading;
    }

    public Pose2d(double x, double y, double rotation) {
        translation_ = new Translation2d(x, y);
        rotation_ = Rotation2d.fromDegrees(rotation);
        chassisHeading_ = Rotation2d.identity();
    }

    public Pose2d(double x, double y, double rotation, double chassisHeading) {
        translation_ = new Translation2d(x, y);
        rotation_ = Rotation2d.fromDegrees(rotation);
        chassisHeading_ = Rotation2d.fromDegrees(chassisHeading);
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation) {
        translation_ = translation;
        rotation_ = rotation;
        chassisHeading_ = Rotation2d.identity();
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation, final Rotation2d chassisHeading) {
        translation_ = translation;
        rotation_ = rotation;
        chassisHeading_ = chassisHeading;
    }

    public Pose2d(final Pose2d other) {
        translation_ = new Translation2d(other.translation_);
        rotation_ = new Rotation2d(other.rotation_);
        chassisHeading_ = new Rotation2d(other.chassisHeading_);
    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static Pose2d exp(final Twist2d delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double sin_heading = Math.sin(delta.dheading);
        double cos_heading = Math.cos(delta.dheading);
        double s, c;
        if (Math.abs(delta.dtheta) < kEps) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation2d(cos_theta, sin_theta, false),
                new Rotation2d(cos_heading, sin_heading, false)
        );
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double dheading = transform.getChassisHeading().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist2d(translation_part.x(), translation_part.y(), dtheta, dheading);
    }

    @Override
    public Translation2d getTranslation() {
        return translation_;
    }

    @Override
    public Rotation2d getRotation() {
        return rotation_;
    }

    @Override
    public Rotation2d getChassisHeading(){
        return chassisHeading_;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    @Override
    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                rotation_.rotateBy(other.rotation_),
                chassisHeading_.rotateBy(other.chassisHeading_));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2d inverse() {
        Rotation2d rotation_inverted = rotation_.inverse();
        Rotation2d chassisHeading_inverted = chassisHeading_.inverse();
        return new Pose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted, chassisHeading_inverted);
    }

    public Pose2d normal() {
        return new Pose2d(translation_, rotation_.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation2d intersection(final Pose2d other) {
        final Rotation2d other_rotation = other.getRotation();
        if (rotation_.isParallel(other_rotation)) {
            // Lines are parallel.
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation_.cos()) < Math.abs(other_rotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final Pose2d other) {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2d twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final Pose2d other, double epsilon) {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation(), epsilon * 2)
                && getChassisHeading().isParallel(other.getChassisHeading(), epsilon * 2);
    }

    private static Translation2d intersectionInternal(final Pose2d a, final Pose2d b) {
        final Rotation2d a_r = a.getRotation();
        final Rotation2d b_r = b.getRotation();
        final Translation2d a_t = a.getTranslation();
        final Translation2d b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                / (a_r.sin() - a_r.cos() * tan_b);
        if (Double.isNaN(t)) {
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2d interpolate(final Pose2d other, double x) {
        if (x <= 0) {
            return new Pose2d(this);
        } else if (x >= 1) {
            return new Pose2d(other);
        }
        final Twist2d twist = Pose2d.log(inverse().transformBy(other));
        return transformBy(Pose2d.exp(twist.scaled(x)));
    }

    @Override
    public String toString() {
        return "T:" + translation_.toString() + ", R:" + rotation_.toString() + ", C:" + chassisHeading_.toString();
    }

    @Override
    public String toCSV() {
        return translation_.toCSV() + "," + rotation_.toCSV() + "," + chassisHeading_.toCSV();
    }

    @Override
    public double distance(final Pose2d other) {
        return Pose2d.log(inverse().transformBy(other)).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Pose2d)) {
            return false;
        }

        return epsilonEquals((Pose2d) other, Util.kEpsilon);
    }

    public boolean equals(final Object other, double epsilon) {
        if (!(other instanceof Pose2d)) {
            return false;
        }

        return epsilonEquals((Pose2d) other, epsilon);
    }

    @Override
    public Pose2d getPose() {
        return this;
    }

    @Override
    public Pose2d mirror() {
        return new Pose2d(new Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse(), getChassisHeading().inverse());
    }
}
