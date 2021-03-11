package com.team254.lib.util;

import com.team254.lib.geometry.Rotation2d;

import java.text.DecimalFormat;

/**
 * A drivetrain signal containing the speed and azimuth for each wheel
 */
public class DriveSignal {
    public static final double[] ZERO_SPEED = new double[]{0, 0, 0, 0};
    public static final Rotation2d[] ZERO_AZIMUTH = new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()};

    public static final DriveSignal NEUTRAL = new DriveSignal(ZERO_SPEED, ZERO_AZIMUTH, false);
    public static final DriveSignal BRAKE = new DriveSignal(ZERO_SPEED, ZERO_AZIMUTH, true);

    private double[] mWheelSpeeds;
    private Rotation2d[] mWheelAzimuths; // Radians!
    private boolean mBrakeMode;

    public DriveSignal() {
        this(ZERO_SPEED, ZERO_AZIMUTH, false);
    }

    public DriveSignal(double[] wheelSpeeds, Rotation2d[] wheelAzimuths, boolean brakeMode) {
        mWheelSpeeds = wheelSpeeds;
        mWheelAzimuths = wheelAzimuths;
        mBrakeMode = brakeMode;
    }

    public double[] getWheelSpeeds() {
        return mWheelSpeeds;
    }

    public Rotation2d[] getWheelAzimuths() {
        return mWheelAzimuths;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    public static DriveSignal fromTank(double left, double right) {
        return new DriveSignal(
            new double[] {left, right, left, right},
            ZERO_AZIMUTH,
            false
        );
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
