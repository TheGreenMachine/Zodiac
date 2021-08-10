package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.RobotState;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.SwerveDrivetrain;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveHelper;
import com.team254.lib.util.SwerveDriveSignal;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public abstract class Drive extends Subsystem implements SwerveDrivetrain, PidProvider {

    private static final String NAME = "drivetrain";

    private static Drive INSTANCE;

    // Components
    private final LedManager ledManager = LedManager.getInstance();
    private final PigeonIMU mPigeon;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    // control states
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private final RobotState mRobotState = RobotState.getInstance();

    // Odometry variables
    private Pose2d pose = Pose2d.identity();
    private Pose2d startingPosition = Pose2d.identity();
    private double lastUpdateTimestamp = 0;

    // Path control variables
    boolean hasStartedFollowing = false;
    boolean hasFinishedPath = false;

    public boolean hasFinishedPath() {
        return hasFinishedPath;
    }

    // hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private double openLoopRampRate;

    private PeriodicIO mPeriodicIO;
    private boolean mOverrideTrajectory = false;

    private boolean isSlowMode;
    private double rotationScalar = 1;
    private boolean robotCentric = false;
    private SendableChooser<DriveHelper> driveHelperChooser;

    // Simulator
    private final Field2d fieldSim = new Field2d();
    private double gyroDrift;
    private final double robotWidthTicks = inchesPerSecondToTicksPer100ms(Constants.kDriveWheelTrackWidthInches) * Math.PI;

    // Constants
    public static final double DRIVE_ENCODER_PPR = factory.getConstant(NAME, "encPPR");
    public static final List<Translation2d> ZERO_DRIVE_VECTOR = List.of(
        Translation2d.identity(),
        Translation2d.identity(),
        Translation2d.identity(),
        Translation2d.identity()
    );

    private Drive() {
        super(NAME);
        mPeriodicIO = new PeriodicIO();

        setOpenLoopRampRate(Constants.kOpenLoopRampRate);

        mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId", -1));

        mPigeon.configFactoryDefault();

        setOpenLoop(SwerveDriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);
    }

    public double getHeadingDegrees() {
        return mPeriodicIO.gyro_heading.getDegrees();
    }

    public abstract double getDesiredHeading();

    public abstract double getHeadingError();

    @Override
    public double getKP() {
        return factory.getConstant(NAME, "kP");
    }

    @Override
    public double getKI() {
        return factory.getConstant(NAME, "kI");
    }

    @Override
    public double getKD() {
        return factory.getConstant(NAME, "kD");
    }

    @Override
    public double getKF() {
        return factory.getConstant(NAME, "kF");
    }

    public static class PeriodicIO {

        // INPUTS
        public double timestamp;
        public Rotation2d gyro_heading = Rotation2d.identity();
        // no_offset = Relative to initial position, unaffected by reset
        public Rotation2d gyro_heading_no_offset = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        private double drive_distance_inches;
        private double velocity_inches_per_second = 0;

        // SWERVE
        public double forward;
        public double rotation;
        public boolean low_power;

        // OUTPUTS

        public Rotation2d desired_heading = Rotation2d.identity();
        TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(
            Pose2dWithCurvature.identity()
        );
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {
                    synchronized (Drive.this) {
                        stop();
                        setBrakeMode(false);
                    }
                    lastUpdateTimestamp = timestamp;
                }

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Drive.this) {
                        mPeriodicIO.timestamp = timestamp;
                        switch (mDriveControlState) {
                            case OPEN_LOOP:
                                updateOpenLoopPeriodic();
                                break;
                            case PATH_FOLLOWING:
                                if (mPathFollower != null) {
                                    //  updatePathFollower(timestamp);
                                    break;
                                }
                            case TRAJECTORY_FOLLOWING:
                                updatePathFollower(timestamp);
                                break;
                            default:
                                System.out.println(
                                    "unexpected drive control state: " +
                                        mDriveControlState
                                );
                                break;
                        }
                    }
                    lastUpdateTimestamp = timestamp;
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    protected abstract void updateOpenLoopPeriodic();

    protected abstract void updatePathFollower(double timestamp);

    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToTicksPer100ms(double inches_per_second) {
        return inchesToRotations(inches_per_second) * DRIVE_ENCODER_PPR / 10.0;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public abstract void setOpenLoop(SwerveDriveSignal signal);

    public abstract void setOpenLoopRampRate(double openLoopRampRate);

    public abstract void setTeleopInputs(
        double forward,
        double strafe,
        double rotation,
        boolean low_power,
        boolean use_heading_controller
    );

    public double getOpenLoopRampRate() {
        return this.openLoopRampRate;
    }

    /**
     * Configure talons for velocity control
     */
    public abstract void setVelocity(List<Translation2d> driveVectors);

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    public abstract void setBrakeMode(boolean on);

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized Rotation2d getHeadingRelativeToInitial() {
        return mPeriodicIO.gyro_heading_no_offset;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset =
            heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.desired_heading = heading;
    }

    public synchronized void resetPigeon() {
        mPigeon.setFusedHeading(0);
    }

    public DriveControlState getDriveControlState() {
        return mDriveControlState;
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (
            mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING
        ) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower =
                new PathFollower(
                    path,
                    reversed,
                    new PathFollower.Parameters(
                        new Lookahead(
                            Constants.kMinLookAhead,
                            Constants.kMaxLookAhead,
                            Constants.kMinLookAheadSpeed,
                            Constants.kMaxLookAheadSpeed
                        ),
                        Constants.kInertiaSteeringGain,
                        Constants.kPathFollowingProfileKp,
                        Constants.kPathFollowingProfileKi,
                        Constants.kPathFollowingProfileKv,
                        Constants.kPathFollowingProfileKffv,
                        Constants.kPathFollowingProfileKffa,
                        Constants.kPathFollowingProfileKs,
                        Constants.kPathFollowingMaxVel,
                        Constants.kPathFollowingMaxAccel,
                        Constants.kPathFollowingGoalPosTolerance,
                        Constants.kPathFollowingGoalVelTolerance,
                        Constants.kPathStopSteeringDistance
                    )
                );
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(ZERO_DRIVE_VECTOR);
        }
    }


    public abstract void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory,
        Rotation2d targetHeading
    );

    public abstract boolean isDoneWithTrajectory();

    public abstract boolean isDoneWithPath();

    public synchronized void forceDoneWithPath() {
        if (
            mDriveControlState == DriveControlState.PATH_FOLLOWING &&
                mPathFollower != null
        ) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (
            mDriveControlState == DriveControlState.PATH_FOLLOWING &&
                mPathFollower != null
        ) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        TRAJECTORY_FOLLOWING,
    }



    @Override
    public void zeroSensors() {
        zeroSensors(Pose2d.identity());
    }

    public abstract void zeroSensors(Pose2d pose);

    public boolean hasPigeonResetOccurred() {
        return mPigeon.hasResetOccurred();
    }

    @Override
    public abstract void stop();

    @Override
    public abstract boolean checkSystem();

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Drive/ControlState",
            () -> this.getDriveControlState().toString(),
            null
        );

        driveHelperChooser = new SendableChooser<>();
        driveHelperChooser.addOption("Cheesy Drive", DriveHelper.CHEESY);
        driveHelperChooser.setDefaultOption("Swerve Classic", DriveHelper.SWERVE_CLASSIC);
        SmartDashboard.putData("Drive Algorithm", driveHelperChooser);
        SmartDashboard.putData("Field", fieldSim);

        SmartDashboard.putNumber("Drive/Vector Direction", 0);
        SmartDashboard.putNumber("Drive/Robot Velocity", 0);
        SmartDashboard.putNumber("Drive/OpenLoopRampRate", this.openLoopRampRate);
        SmartDashboard
            .getEntry("Drive/OpenLoopRampRate")
            .addListener(
                notification -> setOpenLoopRampRate(notification.value.getDouble()),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );

//        SmartDashboard.putBoolean("Drive/TeleopFieldCentric", this.mPeriodicIO.field_relative);
//        SmartDashboard.getEntry("Drive/TeleopFieldCentric")
//            .addListener(
//                notification -> {
//                    this.mPeriodicIO.field_relative = notification.value.getBoolean();
//                },
//                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
//            );

        SmartDashboard.putBoolean("Drive/Zero Sensors", false);
        SmartDashboard
            .getEntry("Drive/Zero Sensors")
            .addListener(
                entryNotification -> {
                    if (entryNotification.value.getBoolean()) {
                        zeroSensors();
                        entryNotification.getEntry().setBoolean(false);
                    }
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}
