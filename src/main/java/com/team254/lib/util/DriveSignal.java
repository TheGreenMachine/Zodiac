package com.team254.lib.util;

import com.team254.lib.geometry.Rotation2d;

import java.text.DecimalFormat;

/**
 * A drivetrain signal containing the speed and azimuth for each wheel
 */
public class DriveSignal {
    private double[] mWheelSpeeds;
    private Rotation2d[] mWheelAzimuths; // Radians!
    private boolean mBrakeMode;

    public DriveSignal() {
        this(new double[]{0, 0, 0, 0}, new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()}, false);
    }

    public DriveSignal(double[] wheelSpeeds, Rotation2d[] wheelAzimuths, boolean brakeMode) {
        mWheelSpeeds = wheelSpeeds;
        mWheelAzimuths = wheelAzimuths;
        mBrakeMode = brakeMode;
    }

    public static final DriveSignal NEUTRAL = new DriveSignal(new double[]{0, 0, 0, 0}, new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()}, false);
    public static final DriveSignal BRAKE = new DriveSignal(new double[]{0, 0, 0, 0}, new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()}, true);

    public double[] getWheelSpeeds() {
        return mWheelSpeeds;
    }

    public Rotation2d[] getWheelAzimuths() {
        return mWheelAzimuths;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        String ret_val = "DriveSignal - \n";
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        for (int i = 0; i < mWheelSpeeds.length; i++) {
            ret_val += "\tWheel " + i + ": Speed - " + mWheelSpeeds[i] + ", Azimuth - " + fmt.format(mWheelAzimuths[i].getDegrees()) + " deg\n";
        }

        return ret_val;
    }
}
