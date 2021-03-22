package com.team1816.lib.subsystems;

import badlog.lib.BadLog;

public class DrivetrainLogger {

    public static void init(TrackableDrivetrain drivetrain) {
        BadLog.createTopic(
            "Drivetrain/LeftActVel",
            "NativeUnits",
            drivetrain::getLeftVelocityNativeUnits,
            "hide",
            "join:Drivetrain/Velocities"
        );
        BadLog.createTopic(
            "Drivetrain/RightActVel",
            "NativeUnits",
            drivetrain::getRightVelocityNativeUnits,
            "hide",
            "join:Drivetrain/Velocities"
        );
        BadLog.createTopic(
            "Drivetrain/LeftVel",
            "NativeUnits",
            drivetrain::getLeftVelocityDemand,
            "hide",
            "join:Drivetrain/Velocities"
        );
        BadLog.createTopic(
            "Drivetrain/RightVel",
            "NativeUnits",
            drivetrain::getRightVelocityDemand,
            "hide",
            "join:Drivetrain/Velocities"
        );
        BadLog.createTopic(
            "Drivetrain/LeftError",
            "NativeUnits",
            drivetrain::getLeftVelocityError,
            "hide",
            "join:Drivetrain/VelocityError"
        );
        BadLog.createTopic(
            "Drivetrain/RightError",
            "NativeUnits",
            drivetrain::getRightVelocityError,
            "hide",
            "join:Drivetrain/VelocityError"
        );
        BadLog.createTopic(
            "Drivetrain/LeftDistance",
            "Inches",
            drivetrain::getLeftEncoderDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
        BadLog.createTopic(
            "Drivetrain/RightDistance",
            "Inches",
            drivetrain::getRightEncoderDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
        BadLog.createTopic(
            "Drivetrain/ActualHeading",
            "Angle",
            drivetrain::getHeadingDegrees,
            "hide",
            "join:Drivetrain/Heading"
        );
        BadLog.createTopic(
            "Drivetrain/Heading",
            "Angle",
            drivetrain::getDesiredHeading,
            "hide",
            "join:Drivetrain/Heading"
        );
    }

    public static void initSwerve(SwerveDrivetrain drivetrain) {
        for (ISwerveModule module : drivetrain.getSwerveModules()) {
            var name = module.getName();
            var prefix = "Drivetrain/" + name;

            // Azimuth
            BadLog.createTopic(
                prefix + "AzimuthPosition",
                "ticks",
                module::getAzimuthPosition,
                "hide",
                "join:Drivetrain/AzimuthPosition"
            );
            BadLog.createTopic(
                prefix + "AzimuthVelocity",
                "ticks",
                module::getAzimuthVelocity,
                "hide",
                "join:Drivetrain/AzimuthVelocity"
            );
            BadLog.createTopic(
                prefix + "AzimuthError",
                "ticks",
                module::getAzimuthError,
                "hide",
                "join:Drivetrain/AzimuthError"
            );

            // Drive
            BadLog.createTopic(
                prefix + "DriveVelocity",
                "ticks",
                module::getDriveVelocity,
                "hide",
                "join:Drivetrain/DriveVelocity"
            );
            BadLog.createTopic(
                prefix + "DriveError",
                "ticks",
                module::getDriveError,
                "hide",
                "join:Drivetrain/DriveError"
            );
        }
    }
}
