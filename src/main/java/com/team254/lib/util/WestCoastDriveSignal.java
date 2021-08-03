package com.team254.lib.util;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class WestCoastDriveSignal {
    private final double mLeftMotor;
    private final double mRightMotor;
    private final boolean mBrakeMode;

    public WestCoastDriveSignal(double left, double right) {
        this(left, right, false);
    }

    public WestCoastDriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static WestCoastDriveSignal fromControls(double throttle, double turn) {
        return new WestCoastDriveSignal(throttle - turn, throttle + turn);
    }

    public static final WestCoastDriveSignal NEUTRAL = new WestCoastDriveSignal(0, 0);
    public static final WestCoastDriveSignal BRAKE = new WestCoastDriveSignal(0, 0, true);

    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}
