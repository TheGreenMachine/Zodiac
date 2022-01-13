package com.team1816.lib.geometry;

import com.team254.lib.geometry.*;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;

public class SwervePose implements IPose2d<SwervePose> {

    protected static final SwervePose kIdentity = new SwervePose();
    public static SwervePose identity() {
        return kIdentity;
    }

    protected final Pose2d pose2d;
    private Rotation2d chassisHeading = Rotation2d.identity();

    public SwervePose() {
        pose2d = new Pose2d();
    }

    public SwervePose(Pose2d pose2d, Rotation2d chassisHeading){
        this.pose2d = pose2d;
        this.chassisHeading = chassisHeading;
    }

    public SwervePose(double x, double y, final Rotation2d rotation, final Rotation2d chassisHeading) {
        pose2d = new Pose2d(x, y, rotation);
        this.chassisHeading = chassisHeading;
    }

    public SwervePose(double x, double y, double rotation, double chassisHeading) {
        pose2d = new Pose2d(x, y, rotation);
        this.chassisHeading = Rotation2d.fromDegrees(chassisHeading);
    }

    public SwervePose(final Translation2d translation, final Rotation2d rotation, final Rotation2d chassisHeading) {
        pose2d = new Pose2d(translation, rotation);
        this.chassisHeading = chassisHeading;
    }

    public SwervePose(final Translation2d translation, final Rotation2d rotation) {
        pose2d = new Pose2d(translation, rotation);
    }

    public SwervePose(final SwervePose other) {
        pose2d = new Pose2d(other.getPose());
        this.chassisHeading = new Rotation2d(other.chassisHeading);
    }

    @Override
    public Pose2d getPose() {
        return this.pose2d;
    }

    @Override
    public SwervePose transformBy(Pose2d transform) {
        return new SwervePose(pose2d.transformBy(transform), chassisHeading);
    }

    @Override
    public SwervePose mirror() {
        return new SwervePose(pose2d.mirror(), chassisHeading.inverse());
    }

    @Override
    public Rotation2d getRotation() {
        return pose2d.getRotation();
    }

    @Override
    public Translation2d getTranslation() {
        return pose2d.getTranslation();
    }

    public Rotation2d getChassisHeading() {
        return chassisHeading;
    }

    @Override
    public double distance(SwervePose other) {
        return pose2d.distance(other.getPose());
    }

    @Override
    public SwervePose interpolate(SwervePose other, double x) {
        return new SwervePose(
            pose2d.interpolate(other.getPose(), x),
            chassisHeading.interpolate(other.getChassisHeading(), x)
        );
    }

    public static SwervePose fromTranslation(final Translation2d translation) {
        return new SwervePose(translation, new Rotation2d(), new Rotation2d());
    }

    public static SwervePose fromRotation(final Rotation2d rotation) {
        return new SwervePose(new Translation2d(), rotation, new Rotation2d());
    }

    @Override
    public String toString() {
        return pose2d.toString() + ", C: " + chassisHeading.toString();
    }

    @Override
    public String toCSV() {
        return pose2d.toCSV() + "," + chassisHeading.toCSV();
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof SwervePose)) {
            return false;
        }
        var swervePose = (SwervePose)other;
        return pose2d.equals(swervePose.pose2d) && chassisHeading.equals(swervePose.chassisHeading);
    }
}

