package com.team254.lib.util;

import com.team1816.frc2020.Constants;
import com.team1816.frc2020.subsystems.SwerveDrive;
import com.team1816.frc2020.subsystems.SwerveModule;
import com.team254.lib.geometry.Rotation2d;

import java.text.DecimalFormat;
import java.util.Arrays;

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

    public DriveSignal(double left, double right) {
        mWheelSpeeds = new double[4];
        mWheelSpeeds[SwerveModule.kFrontLeft] = left;
        mWheelSpeeds[SwerveModule.kBackLeft] = left;
        mWheelSpeeds[SwerveModule.kFrontRight] = right;
        mWheelSpeeds[SwerveModule.kBackRight] = right;

        mWheelAzimuths = ZERO_AZIMUTH;
        mBrakeMode = false;
    }

    public DriveSignal(double[] wheelSpeeds, Rotation2d[] wheelAzimuths, boolean brakeMode) {
        mWheelSpeeds = wheelSpeeds;
        mWheelAzimuths = wheelAzimuths;
        mBrakeMode = brakeMode;
    }

    public double[] getWheelSpeeds() {
        return mWheelSpeeds;
    }

    public DriveSignal toVelocity() {
        return new DriveSignal(
            Arrays.stream(this.mWheelSpeeds)
                .map(x ->
                    x * SwerveDrive.inchesPerSecondToTicksPer100ms(Constants.kPathFollowingMaxVel)
                )
                .toArray(),
            this.mWheelAzimuths,
            this.mBrakeMode
        );
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
