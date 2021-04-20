package com.team1816.frc2020.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1816.frc2020.AutoModeSelector;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.RobotState;
import com.team1816.frc2020.planners.DriveMotionPlanner;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.SwerveDrivetrain;
import com.team1816.lib.subsystems.TrackableDrivetrain;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SwerveDriveHelper;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive
    extends Subsystem
    implements SwerveDrivetrain, TrackableDrivetrain, PidProvider {

    private static Drive mInstance;
    private static final String NAME = "drivetrain";
    private static double DRIVE_ENCODER_PPR;

    // Components
    private final SwerveModule[] mModules = new SwerveModule[4];
    private final LedManager ledManager = LedManager.getInstance();

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    // control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;

    // hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private double mLastDriveCurrentSwitchTime = -1;
    private double openLoopRampRate;
    private BadLog mLogger;

    private PeriodicIO mPeriodicIO;
    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private boolean isSlowMode;
    private SendableChooser<DriveHelper> driveHelperChooser;

    public static synchronized Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private Drive() {
        super(NAME);
        DRIVE_ENCODER_PPR = factory.getConstant(NAME, "encPPR");
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mModules[SwerveModule.kFrontLeft] = factory.getSwerveModule(NAME, "frontLeft");
        mModules[SwerveModule.kFrontRight] = factory.getSwerveModule(NAME, "frontRight");
        mModules[SwerveModule.kBackLeft] = factory.getSwerveModule(NAME, "backLeft");
        mModules[SwerveModule.kBackRight] = factory.getSwerveModule(NAME, "backRight");

        setOpenLoopRampRate(Constants.kOpenLoopRampRate);

        mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId", -1));

        mPigeon.configFactoryDefault();

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);

        mMotionPlanner = new DriveMotionPlanner();
    }

    public double getHeadingDegrees() {
        return mPeriodicIO.gyro_heading.getDegrees();
    }

    public double getDesiredHeading() {
        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            return mPeriodicIO.path_setpoint.state().getRotation().getDegrees();
        }
        return mPeriodicIO.desired_heading.getDegrees();
    }

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
        public double left_position_ticks;
        public double right_position_ticks;
        public double left_velocity_ticks_per_100ms;
        public double right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        // no_offset = Relative to initial position, unaffected by reset
        public Rotation2d gyro_heading_no_offset = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        double left_error;
        double right_error;

        // SWERVE
        public double forward;
        public double strafe;
        public double rotation;
        public boolean low_power;
        public boolean field_relative;
        public boolean use_heading_controller;

        // OUTPUTS
        public double[] wheel_speeds = new double[] { 0, 0, 0, 0 };
        public Rotation2d[] wheel_azimuths = new Rotation2d[] {
            Rotation2d.identity(),
            Rotation2d.identity(),
            Rotation2d.identity(),
            Rotation2d.identity(),
        };
        public Rotation2d desired_heading = Rotation2d.identity();
        TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(
            Pose2dWithCurvature.identity()
        );
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (mPigeon.getLastError() != ErrorCode.OK) {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            //    System.out.println("Pigeon error detected, maybe reinitialized");
        }
        mPeriodicIO.gyro_heading_no_offset =
            Rotation2d.fromDegrees(mPigeon.getFusedHeading());
        mPeriodicIO.gyro_heading =
            mPeriodicIO.gyro_heading_no_offset.rotateBy(mGyroOffset);
        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
        for (SwerveModule module : mModules) {
            module.readPeriodicInputs();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; i++) {
            if (mModules[i] != null) {
                mModules[i].setOpenLoop(
                        mPeriodicIO.wheel_speeds[i],
                        mPeriodicIO.wheel_azimuths[i]
                    );
                mModules[i].writePeriodicOutputs();
            }
        }
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
                }

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Drive.this) {
                        switch (mDriveControlState) {
                            case OPEN_LOOP:
                                var driveHelper = driveHelperChooser.getSelected();
                                setOpenLoop(
                                    driveHelper.calculateDriveSignal(
                                        mPeriodicIO.forward,
                                        mPeriodicIO.strafe,
                                        mPeriodicIO.rotation,
                                        mPeriodicIO.low_power,
                                        mPeriodicIO.field_relative,
                                        mPeriodicIO.use_heading_controller
                                    )
                                );
                                break;
                            case PATH_FOLLOWING:
                                if (mPathFollower != null) {
                                    if (Constants.kIsBadlogEnabled) {
                                        // mLogger.updateTopics();
                                        // mLogger.log();
                                    }
                                    updatePathFollower(timestamp);
                                }
                            case TRAJECTORY_FOLLOWING:
                                if (Constants.kIsBadlogEnabled) {
                                    // mLogger.updateTopics();
                                    // mLogger.log();
                                }
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
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToTicksPer100ms(double inches_per_second) {
        return inchesToRotations(inches_per_second) * DRIVE_ENCODER_PPR / 10.0;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        System.out.println(mPeriodicIO.wheel_speeds);
        mPeriodicIO.wheel_speeds = signal.getWheelSpeeds();
        mPeriodicIO.wheel_azimuths = signal.getWheelAzimuths();
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
        for (SwerveModule module : mModules) {
            module.setOpenLoopRampRate(openLoopRampRate);
        }
    }

    public void setTeleopInputs(double forward, double strafe, double rotation, boolean low_power, boolean field_relative, boolean use_heading_controller) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        mPeriodicIO.low_power = low_power;
        mPeriodicIO.field_relative = field_relative;
        mPeriodicIO.use_heading_controller = use_heading_controller;
    }

    public double getOpenLoopRampRate() {
        return this.openLoopRampRate;
    }

    /**
     * Configure talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("Switching to Velocity");
            // mLeftMaster.selectProfileSlot(0, 0);
            // mRightMaster.selectProfileSlot(0, 0);
            // mLeftMaster.configNeutralDeadband(0.0, 0);
            // mRightMaster.configNeutralDeadband(0.0, 0);
        }
        // mPeriodicIO.left_demand = signal.getLeft();
        // mPeriodicIO.right_demand = signal.getRight();
        // mPeriodicIO.left_feedforward = feedforward.getLeft();
        // mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    public void setLogger(BadLog logger) {
        mLogger = logger;
    }

    public synchronized void setBrakeMode(boolean on) {
        for (SwerveModule module : mModules) {
            module.setDriveBrakeMode(on);
        }
    }

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

    public synchronized double[] getModuleVelocities() {
        double[] ret_val = new double[mModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = mModules[i].getLinearVelocity();
        }

        return ret_val;
    }

    public synchronized Rotation2d[] getModuleAzimuths() {
        Rotation2d[] ret_val = new Rotation2d[mModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = mModules[i].getAngle();
        }

        return ret_val;
    }

    public synchronized void resetPigeon() {
        mPigeon.setFusedHeading(0);
    }

    /* public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    } */

    public DriveControlState getDriveControlState() {
        return mDriveControlState;
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    @Override
    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    @Override
    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    @Override
    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(
            getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR
        );
    }

    @Override
    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (
            Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0
        );
    }

    public double getAngularVelocity() {
        return (
            (getRightLinearVelocity() - getLeftLinearVelocity()) /
            Constants.kDriveWheelTrackWidthInches
        );
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
            setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
        }
    }

    public synchronized void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory
    ) {
        if (mMotionPlanner != null) {
            System.out.println("Now setting trajectory");
            setBrakeMode(true);
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (
            mMotionPlanner == null ||
            mDriveControlState != DriveControlState.TRAJECTORY_FOLLOWING
        ) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public synchronized boolean isDoneWithPath() {
        if (
            mDriveControlState == DriveControlState.PATH_FOLLOWING &&
            mPathFollower != null
        ) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

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

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            // RobotState robot_state = RobotState.getInstance();
            // Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            // Twist2d command = mPathFollower.update(
            //     timestamp,
            //     field_to_vehicle,
            //     robot_state.getDistanceDriven(),
            //     robot_state.getPredictedVelocity().dx
            // );
            // if (!mPathFollower.isFinished()) {
            //     DriveSignal setpoint = Kinematics.inverseKinematics(command);
            //     setVelocity(
            //         new DriveSignal(
            //             inchesPerSecondToTicksPer100ms(setpoint.getLeft()),
            //             inchesPerSecondToTicksPer100ms(setpoint.getRight())
            //         ),
            //         new DriveSignal(0, 0)
            //     );
            // } else {
            //     if (!mPathFollower.isForceFinished()) {
            //         setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
            //     }
            // }
        } else if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            DriveMotionPlanner.Output output = mMotionPlanner.update(
                timestamp,
                RobotState.getInstance().getFieldToVehicle(timestamp)
            );

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();
            // if (!mOverrideTrajectory) {
            //     setVelocity(
            //         new DriveSignal(
            //             radiansPerSecondToTicksPer100ms(output.left_velocity),
            //             radiansPerSecondToTicksPer100ms(output.right_velocity)
            //         ),
            //         new DriveSignal(
            //             output.left_feedforward_voltage / 12.0,
            //             output.right_feedforward_voltage / 12.0
            //         )
            //     );
            //
            //     mPeriodicIO.left_accel =
            //         radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
            //     mPeriodicIO.right_accel =
            //         radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            // } else {
            //     setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
            //     mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            // }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
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
    public SwerveModule[] getSwerveModules() {
        return mModules;
    }

    @Override
    public void zeroSensors() {
        System.out.println("Zeroing drive sensors!");
        resetPigeon();
        setHeading(Rotation2d.identity());
        for (int i = 0; i < mModules.length; i++) {
            if (mModules != null && mModules[i] != null) {
                mModules[i].zeroSensors();
            }
        }
        if (mPigeon.getLastError() != ErrorCode.OK) {
            // BadLog.createValue("PigeonErrorDetected", "true");
            System.out.println(
                "Error detected with Pigeon IMU - check if the sensor is present and plugged in!"
            );
            System.out.println("Defaulting to drive straight mode");
            AutoModeSelector.getInstance().setHardwareFailure(true);
        } else {
            AutoModeSelector.getInstance().setHardwareFailure(false);
        }
    }

    public boolean hasPigeonResetOccurred() {
        return mPigeon.hasResetOccurred();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        boolean modulesPassed = true;
        for (SwerveModule module : mModules) {
            modulesPassed = modulesPassed && module.checkSystem();
        }

        boolean checkPigeon = mPigeon == null;

        System.out.println(modulesPassed && checkPigeon);
        return modulesPassed;
    }

    @Override
    public double getLeftVelocityDemand() {
        return 0;
    }

    @Override
    public double getRightVelocityDemand() {
        return 0;
    }

    @Override
    public double getLeftVelocityError() {
        return mPeriodicIO.left_error;
    }

    @Override
    public double getRightVelocityError() {
        return mPeriodicIO.right_error;
    }

    public double getRightDriveTicks() {
        return mPeriodicIO.right_position_ticks;
    }

    public double getLeftDriveTicks() {
        return mPeriodicIO.left_position_ticks;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Right Drive Distance",
            this::getRightEncoderDistance,
            null
        );
        builder.addDoubleProperty("Right Drive Ticks", this::getRightDriveTicks, null);
        builder.addDoubleProperty(
            "Left Drive Distance",
            this::getLeftEncoderDistance,
            null
        );
        builder.addDoubleProperty("Left Drive Ticks", this::getLeftDriveTicks, null);
        builder.addStringProperty(
            "Drive/ControlState",
            () -> this.getDriveControlState().toString(),
            null
        );
        builder.addBooleanProperty(
            "Drive/PigeonIMU State",
            () -> this.mPigeon.getLastError() == ErrorCode.OK,
            null
        );

        driveHelperChooser = new SendableChooser<>();
        driveHelperChooser.setDefaultOption("Swerve Classic", DriveHelper.SWERVE_CLASSIC);
        SmartDashboard.putData("Drive Algorithm", driveHelperChooser);

        SmartDashboard.putNumber("Drive/OpenLoopRampRate", this.openLoopRampRate);
        SmartDashboard
            .getEntry("Drive/OpenLoopRampRate")
            .addListener(
                notification -> {
                    setOpenLoopRampRate(notification.value.getDouble());
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );

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
