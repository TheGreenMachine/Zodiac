package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import java.text.DecimalFormat;

public class Pose2dWithCurvature<T extends IPose2d<T>> implements IPose2d<Pose2dWithCurvature<T>>, ICurvature<Pose2dWithCurvature<T>> {
    protected static final Pose2dWithCurvature<Pose2d> kIdentity = new Pose2dWithCurvature<Pose2d>(Pose2d.identity());

    public static Pose2dWithCurvature<Pose2d> identity() {
        return kIdentity;
    }

    protected final T pose_;
    protected final double curvature_;
    protected final double dcurvature_ds_;

    public Pose2dWithCurvature(final T pose) {
        pose_ = pose;
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithCurvature(final T pose, double curvature) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithCurvature(final T pose, double curvature, double dcurvature_ds) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    @Override
    public final Pose2d getPose() {
        return (Pose2d) pose_;
    }

    @Override
    public Pose2dWithCurvature<T> transformBy(Pose2d transform) {
        return new Pose2dWithCurvature<T>(pose_.transformBy(transform), getCurvature(), getDCurvatureDs());
    }

    @Override
    public Pose2dWithCurvature<T> mirror() {
        return new Pose2dWithCurvature<T>(pose_.mirror(), -getCurvature(), -getDCurvatureDs());
    }

    @Override
    public double getCurvature() {
        return curvature_;
    }

    @Override
    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    @Override
    public final Translation2d getTranslation() {
        return pose_.getTranslation();
    }

    @Override
    public final Rotation2d getRotation() {
        return pose_.getRotation();
    }

    @Override
    public Pose2dWithCurvature<T> interpolate(final Pose2dWithCurvature<T> other, double x) {
        return new Pose2dWithCurvature<T>(pose_.interpolate(other.pose_, x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    @Override
    public double distance(final Pose2dWithCurvature<T> other) {
        return pose_.distance(other.pose_);
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithCurvature)) {
            return false;
        }

        Pose2dWithCurvature<?> p2dwc = (Pose2dWithCurvature<?>) other; // wtf
        return pose_.equals(p2dwc.pose_)
            && Util.epsilonEquals(getCurvature(), p2dwc.getCurvature())
            && Util.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return pose_.toString() + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return pose_.toCSV() + "," + fmt.format(getCurvature()) + "," + fmt.format(getDCurvatureDs());
    }
}
