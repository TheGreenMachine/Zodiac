package com.team254.lib.geometry;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;

import java.text.DecimalFormat;

import static com.team254.lib.util.Util.kEpsilon;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle
 * (cosine and sine).
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Rotation2d extends edu.wpi.first.wpilibj.geometry.Rotation2d implements IRotation2d<Rotation2d> {
    protected static final Rotation2d kIdentity = new Rotation2d();

    public static Rotation2d identity() {
        return kIdentity;
    }

    protected double cos_angle_ = Double.NaN;
    protected double sin_angle_ = Double.NaN;
    protected double radians_ = Double.NaN;
    protected double theta_degrees = 0;
    protected double theta_radians = 0;

    protected Rotation2d(double x, double y, double radians) {
        cos_angle_ = x;
        sin_angle_ = y;
        radians_ = radians;
    }

    public Rotation2d() {
        this(1.0, 0.0, 0.0);
    }

    public Rotation2d(double radians, boolean normalize) {
        if (normalize) {
            radians = WrapRadians(radians);
        }
        radians_ = radians;
    }

    public Rotation2d(double x, double y, boolean normalize) {
        if (normalize) {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object
            // we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                sin_angle_ = y / magnitude;
                cos_angle_ = x / magnitude;
            } else {
                sin_angle_ = 0.0;
                cos_angle_ = 1.0;
            }
        } else {
            cos_angle_ = x;
            sin_angle_ = y;
        }
        theta_degrees = Math.toDegrees(Math.atan2(sin_angle_, cos_angle_));
    }

    public Rotation2d(final Rotation2d other) {
        cos_angle_ = other.cos_angle_;
        sin_angle_ = other.sin_angle_;
        radians_ = other.radians_;
        theta_degrees = Math.toDegrees(Math.atan2(sin_angle_, cos_angle_));
    }

    public Rotation2d(double theta_degrees){
        cos_angle_ = Math.cos(Math.toRadians(theta_degrees));
        sin_angle_ = Math.sin(Math.toRadians(theta_degrees));
        this.theta_degrees = theta_degrees;
    }

    public Rotation2d(final Translation2d direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }

    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(angle_radians, true);
    }

    public static Rotation2d fromDegrees(double angle_degrees) {
        return new Rotation2d(angle_degrees);
    }

    public double cos() {
        ensureTrigComputed();
        return cos_angle_;
    }

    public double sin() {
        ensureTrigComputed();
        return sin_angle_;
    }

    public double tan() {
        ensureTrigComputed();
        if (Math.abs(cos_angle_) < kEpsilon) {
            if (sin_angle_ >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sin_angle_ / cos_angle_;
    }

    @Override
    public double getRadians() {
        ensureRadiansComputed();
        return radians_;
    }

    @Override
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public double getUnboundedDegrees() {
        return theta_degrees;
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and
     * another rotation.
     *
     * @param other The other rotation. See:
     *              https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation2d rotateBy(final Rotation2d other) {
        if (hasTrig() && other.hasTrig()) {
            return new Rotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                    cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
        } else {
            return fromRadians(getRadians() + other.getRadians());
        }
    }

    public Rotation2d normal() {
        if (hasTrig()) {
            return new Rotation2d(-sin_angle_, cos_angle_, false);
        } else {
            return fromRadians(getRadians() - Math.PI / 2.0);
        }
    }

    /**
     * Based on Team 1323's method of the same name.
     *
     * @return Rotation2d representing the angle of the nearest axis to the angle in standard position
     */
    public Rotation2d nearestPole() {
        double pole_sin = 0.0;
        double pole_cos = 0.0;
        if (Math.abs(cos_angle_) > Math.abs(sin_angle_)) {
            pole_cos = Math.signum(cos_angle_);
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(sin_angle_);
        }
        return new Rotation2d(pole_cos, pole_sin, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    public Rotation2d inverse() {
        if (hasTrig()) {
            return new Rotation2d(cos_angle_, -sin_angle_, false);
        } else {
            return fromRadians(-getRadians());
        }
    }

    public boolean isParallel(final Rotation2d other) {
        if (hasRadians() && other.hasRadians()) {
            return Util.epsilonEquals(radians_, other.radians_)
                    || Util.epsilonEquals(radians_, WrapRadians(other.radians_ + Math.PI));
        } else if (hasTrig() && other.hasTrig()) {
            return Util.epsilonEquals(sin_angle_, other.sin_angle_) && Util.epsilonEquals(cos_angle_, other.cos_angle_);
        } else {
            // Use public, checked version.
            return Util.epsilonEquals(getRadians(), other.getRadians())
                    || Util.epsilonEquals(radians_, WrapRadians(other.radians_ + Math.PI));
        }
    }

    public Translation2d toTranslation() {
        ensureTrigComputed();
        return new Translation2d(cos_angle_, sin_angle_);
    }

    protected double WrapRadians(double radians) {
        final double k2Pi = 2.0 * Math.PI;
        radians = radians % k2Pi;
        radians = (radians + k2Pi) % k2Pi;
        if (radians > Math.PI)
            radians -= k2Pi;
        return radians;
    }

    private boolean hasTrig() {
        return !Double.isNaN(sin_angle_) && !Double.isNaN(cos_angle_);
    }

    private boolean hasRadians() {
        return !Double.isNaN(radians_);
    }

    private void ensureTrigComputed() {
        if (!hasTrig()) {
            if (Double.isNaN(radians_)) {
                DriverStation.reportError("Ensure hasTrig", true);
            }
            sin_angle_ = Math.sin(radians_);
            cos_angle_ = Math.cos(radians_);
        }
    }

    private void ensureRadiansComputed() {
        if (!hasRadians()) {
            if (Double.isNaN(cos_angle_) || Double.isNaN(sin_angle_)) {
                DriverStation.reportError("Ensure hasRadians", true);
            }
            radians_ = Math.atan2(sin_angle_, cos_angle_);
        }
    }

    @Override
    public Rotation2d interpolate(final Rotation2d other, double x) {
        if (x <= 0.0) {
            return new Rotation2d(this);
        } else if (x >= 1.0) {
            return new Rotation2d(other);
        }
        double angle_diff = inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2d.fromRadians(angle_diff * x));
    }

    @Override
    public String toString() {
        return "(" + new DecimalFormat("#0.000").format(getDegrees()) + " deg)";
    }

    @Override
    public String toCSV() {
        return new DecimalFormat("#0.000").format(getDegrees());
    }

    @Override
    public double distance(final Rotation2d other) {
        return inverse().rotateBy(other).getRadians();
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Rotation2d)) {
            return false;
        }

        return distance((Rotation2d) other) < Util.kEpsilon;
    }

    @Override
    public Rotation2d getRotation() {
        return this;
    }
}
