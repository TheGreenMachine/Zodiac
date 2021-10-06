package com.team1816.lib.subsystems;

import badlog.lib.BadLog;

public class DrivetrainLogger {

    public static void init(TrackableDrivetrain drivetrain) {
        var subsystem = (Subsystem) drivetrain;
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
            "Drivetrain/X Desired",
            "Inches",
            drivetrain::getFieldDesiredXDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
         BadLog.createTopic(
            "Drivetrain/Y Desired",
            "Inches",
            drivetrain::getFieldYDesiredYDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
         BadLog.createTopic(
            "Drivetrain/X Actual",
            "Inches",
            drivetrain::getFieldXDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
         BadLog.createTopic(
            "Drivetrain/Y Actual",
            "Inches",
            drivetrain::getFieldYDistance,
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
}
