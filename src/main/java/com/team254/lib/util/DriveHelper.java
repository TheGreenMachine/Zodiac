package com.team254.lib.util;

public interface DriveHelper {
    public static final DriveHelper SWERVE_CLASSIC = new SwerveDriveHelper();

    DriveSignal calculateDriveSignal(
        double forwardInput,
        double strafeInput,
        double rotationInput,
        boolean low_power,
        boolean field_relative,
        boolean use_heading_controller
    );
}