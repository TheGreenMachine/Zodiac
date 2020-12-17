package com.team1816.frc2020;

import com.team1816.lib.hardware.RobotFactory;

public class Constants {
    private static final RobotFactory factory = Robot.getFactory();

    public static final double kLooperDt = factory.getConstant("kLooperDt");
    public static final double kDriveWheelTrackWidthInches = factory.getConstant("trackWidth");
    public static final double kDriveWheelDiameterInches = factory.getConstant("wheelDiameter");

    public static double kTrackScrubFactor = factory.getConstant("kTrackScrubFactor");
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kOpenLoopRampRate = factory.getConstant("drivetrain", "openLoopRampRate");

    // reset button
    public static final int kResetButtonChannel = 4;

    // Control Board
    public static final boolean kUseDriveGamepad = true;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 1;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 0;
    public static final double kJoystickThreshold = 0.04; // deadband

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 12.0; // inches per second
    public static final double kMaxLookAhead = 48.0; // inches
    public static final double kMaxLookAheadSpeed = factory.getConstant("maxVel"); // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;
    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain * our speed in inches per sec
    public static final double kPathFollowingProfileKp = factory.getConstant("drivetrain", "kP");
    public static final double kPathFollowingProfileKi = factory.getConstant("drivetrain", "kI");

    public static final double kPathFollowingGoalPosTolerance = 1.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;
    public static double kCameraFrameRate = 30;
    public static final double kPathFollowingMaxAccel = factory.getConstant("maxAccel");
    public static final double kPathFollowingMaxVel = factory.getConstant("maxVel");

    // Trajectory Generator constants

    // Tuned dynamics
    public static final double kRobotLinearInertia = 10.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2

    /* CONTROL LOOP GAINS */

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  //    public static final double kPathFollowingProfileKv = 0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKv = 0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKffv = 1.0;  // % throttle per inch/s
    public static final double kPathFollowingProfileKffa = 0;  // % throttle per inch/s^2
    public static final double kPathFollowingProfileKs = factory.getConstant("pathKs");;  // % throttle required to break static friction

    public static final boolean kIsBadlogEnabled = factory.getConstant("badLogEnabled") > 0;
    public static final boolean kIsLoggingTeleOp = factory.getConstant("logTeleOp") > 0;
    public static final boolean kIsLoggingAutonomous = factory.getConstant("logAuto") > 0;

    public static final boolean kUseAutoAim = factory.getConstant("useAutoAim") > 0;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

}
