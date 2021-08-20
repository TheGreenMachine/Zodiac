package com.team1816.lib.subsystems;

import badlog.lib.BadLog;

public class DrivetrainLogger {

    public static void initDifferential(DifferentialDrivetrain drivetrain) {
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
        baseInit(drivetrain);
    }

    public static void initSwerve(SwerveDrivetrain drivetrain) {
        for (ISwerveModule module : drivetrain.getSwerveModules()) {
            var name = module.getSubsystemName();
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
                prefix + "AzimuthPositionDemand",
                "ticks",
                module::getAzimuthPositionDemand,
                "hide",
                "join:Drivetrain/AzimuthPosition"
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
                prefix + "DriveVelocityDemand",
                "ticks",
                module::getDriveVelocityDemand,
                "hide",
                "join:Drivetrain/DriveVelocity"
            );
            BadLog.createTopic(
                prefix + "DriveDistance",
                "ticks",
                module::getDriveDistance,
                "hide",
                "join:Drivetrain/DriveDistance"
            );
            BadLog.createTopic(
                prefix + "DriveError",
                "ticks",
                module::getDriveError,
                "hide",
                "join:Drivetrain/DriveError"
            );
        }
        baseInit(drivetrain);
    }

    private static void baseInit(TrackableDrivetrain drivetrain) {
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
//
//        BadLog.createTopic(
//            "Drivetrain/HeadingError",
//            "Angle",
//            drivetrain::getHeadingError
//        );
    }

    public static void init(TrackableDrivetrain drivetrain){
        if(drivetrain instanceof SwerveDrivetrain)
            DrivetrainLogger.initSwerve((SwerveDrivetrain) drivetrain);
        else if(drivetrain instanceof DifferentialDrivetrain)
            DrivetrainLogger.initDifferential((DifferentialDrivetrain) drivetrain);
        else
            DrivetrainLogger.baseInit(drivetrain);
    }
}
