package com.team1816.frc2020;

public class Constants {
    public static double kLooperDt = Robot.getFactory().getConstant("kLooperDt");
    public static final double kDriveWheelTrackWidthInches = Robot.getFactory().getConstant("trackWidth");
    public static final double kDriveWheelDiameterInches = Robot.getFactory().getConstant("wheelDiameter");

    public static double kTrackScrubFactor = Robot.getFactory().getConstant("kTrackScrubFactor");
    public static int kPCMId = Robot.getFactory().getPcmId();
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kOpenLoopRampRate = Robot.getFactory().getConstant("drivetrain", "openLoopRampRate");

    // reset button
    public static final int kResetButtonChannel = 4;

    // Control Board
    public static final boolean kUseDriveGamepad = true;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 1;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 0;
    public static final double kJoystickThreshold = 0.2;

    // Canifier
    public static int kCanifierWristId = (int) Robot.getFactory().getConstant("canifier", "canId");

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
    public static final double kMaxLookAheadSpeed = Robot.getFactory().getConstant("maxVel"); // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;
    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain * our speed in inches per sec
    public static final double kPathFollowingProfileKp = Robot.getFactory().getConstant("drivetrain", "kP");
    public static final double kPathFollowingProfileKi = Robot.getFactory().getConstant("drivetrain", "kI");

    public static final double kPathFollowingGoalPosTolerance = 1.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;
    public static double kCameraFrameRate = 30;
    public static final double kPathFollowingMaxAccel = Robot.getFactory().getConstant("maxAccel");
    public static final double kPathFollowingMaxVel = Robot.getFactory().getConstant("maxVel");

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
    public static final double kPathFollowingProfileKs = Robot.getFactory().getConstant("pathKs");;  // % throttle required to break static friction

    public static final double kDriveVoltageRampRate = 0.0;



    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

}
