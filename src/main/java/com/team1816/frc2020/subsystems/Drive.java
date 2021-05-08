package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1816.frc2020.AutoModeSelector;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.Kinematics;
import com.team1816.frc2020.RobotState;
import com.team1816.frc2020.planners.DriveMotionPlanner;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.SwerveDrivetrain;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class Drive extends Subsystem implements SwerveDrivetrain, PidProvider {

    private static final String NAME = "drivetrain";

    private static Drive INSTANCE;

    // Components
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final LedManager ledManager = LedManager.getInstance();
    private final PigeonIMU mPigeon;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;
    private final DriveMotionPlanner motionPlanner = DriveMotionPlanner.getInstance();
    private final SwerveHeadingController headingController = SwerveHeadingController.getInstance();

    // control states
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private final RobotState mRobotState = RobotState.getInstance();

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

    private final Field2d fieldSim = new Field2d();

    // Constants
    public static final double DRIVE_ENCODER_PPR = factory.getConstant(NAME, "encPPR");
    public static final List<Translation2d> ZERO_DRIVE_VECTOR = List.of(
        Translation2d.identity(),
        Translation2d.identity(),
        Translation2d.identity(),
        Translation2d.identity()
    );

    public static synchronized Drive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Drive();
        }

        return INSTANCE;
    }

    private Drive() {
        super(NAME);
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        swerveModules[SwerveModule.kFrontLeft] =
            factory.getSwerveModule(
                NAME,
                "frontLeft",
                Constants.kFrontLeftModulePosition
            );
        swerveModules[SwerveModule.kFrontRight] =
            factory.getSwerveModule(
                NAME,
                "frontRight",
                Constants.kFrontRightModulePosition
            );
        swerveModules[SwerveModule.kBackLeft] =
            factory.getSwerveModule(NAME, "backLeft", Constants.kBackLeftModulePosition);
        swerveModules[SwerveModule.kBackRight] =
            factory.getSwerveModule(
                NAME,
                "backRight",
                Constants.kBackRightModulePosition
            );

        setOpenLoopRampRate(Constants.kOpenLoopRampRate);

        mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId", -1));

        mPigeon.configFactoryDefault();

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);
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
        public Rotation2d gyro_heading = Rotation2d.identity();
        // no_offset = Relative to initial position, unaffected by reset
        public Rotation2d gyro_heading_no_offset = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        private double drive_distance_inches;
        private double velocity_inches_per_second = 0;

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
        for (SwerveModule module : swerveModules) {
            module.readPeriodicInputs();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        var rot2d = new edu.wpi.first.wpilibj.geometry.Rotation2d(
            mPeriodicIO.gyro_heading_no_offset.getRadians()
        );
        fieldSim.setRobotPose(
            Units.inches_to_meters(mRobotState.getEstimatedX()),
            Units.inches_to_meters(mRobotState.getEstimatedY()) + 3.5,
            rot2d
        );
        for (int i = 0; i < swerveModules.length; i++) {
            if (swerveModules[i] != null) {
                if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                    // TODO: 5/5/21 fix
                    swerveModules[i].setVelocity(
                            mPeriodicIO.wheel_speeds[i],
                            mPeriodicIO.wheel_azimuths[i]
                        );
                } else if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
                    swerveModules[i].setVelocity(
                            mPeriodicIO.wheel_speeds[i],
                            mPeriodicIO.wheel_azimuths[i]
                        );
                }
                swerveModules[i].writePeriodicOutputs();
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
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

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
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        mPeriodicIO.wheel_speeds = signal.getWheelSpeeds();
        mPeriodicIO.wheel_azimuths = signal.getWheelAzimuths();
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
        for (SwerveModule module : swerveModules) {
            module.setOpenLoopRampRate(openLoopRampRate);
        }
    }

    public void setTeleopInputs(
        double forward,
        double strafe,
        double rotation,
        boolean low_power,
        boolean field_relative,
        boolean use_heading_controller
    ) {
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
    public synchronized void setVelocity(List<Translation2d> driveVectors) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("Switching to Velocity");
        }
        for (int i = 0; i < swerveModules.length; i++) {
            mPeriodicIO.wheel_azimuths[i] = driveVectors.get(i).direction();
            mPeriodicIO.wheel_speeds[i] =
                inchesPerSecondToTicksPer100ms(
                    driveVectors.get(i).norm() * Constants.kPathFollowingMaxVel
                );
        }
    }

    public void updateOdometry(double deltaDistance, double velocity) {
        mPeriodicIO.drive_distance_inches += deltaDistance;
        mPeriodicIO.velocity_inches_per_second = velocity;
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        for (SwerveModule module : swerveModules) {
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
        double[] ret_val = new double[swerveModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = swerveModules[i].getLinearVelocity();
        }

        return ret_val;
    }

    public synchronized Rotation2d[] getModuleAzimuths() {
        Rotation2d[] ret_val = new Rotation2d[swerveModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = swerveModules[i].getAngle();
        }

        return ret_val;
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

    public synchronized void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory
    ) {
        if (motionPlanner != null) {
            System.out.println("Now setting trajectory");
            setBrakeMode(true);
            mOverrideTrajectory = false;
            motionPlanner.reset();
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
            motionPlanner.setTrajectory(trajectory);
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mDriveControlState != DriveControlState.TRAJECTORY_FOLLOWING) {
            return false;
        }
        return motionPlanner.isDone() || mOverrideTrajectory;
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
        var pose = mRobotState.getFieldToVehicle(timestamp);

        headingController.setGoal(pose.getRotation().getUnboundedDegrees());
        double rotationCorrection = headingController.update();

        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            if (!motionPlanner.isDone()) {
                Translation2d driveVector = motionPlanner.update(timestamp, pose);
                //                System.out.println("DRIVE VECTOR" + driveVector);

                mPeriodicIO.forward = driveVector.x();
                mPeriodicIO.strafe = driveVector.y();
                mPeriodicIO.rotation = 0;

                double rotationInput = Util.deadBand(
                    Util.limit(
                        rotationCorrection * rotationScalar * driveVector.norm(),
                        motionPlanner.getMaxRotationSpeed()
                    ),
                    0.01
                );

                mPeriodicIO.error = motionPlanner.error();
                mPeriodicIO.path_setpoint = motionPlanner.setpoint();
                if (!mOverrideTrajectory) {
                    setVelocity(
                        Kinematics.updateDriveVectors(
                            driveVector,
                            rotationInput,
                            pose,
                            robotCentric
                        )
                    );
                }
            } else {
                setVelocity(ZERO_DRIVE_VECTOR);
            }
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
        return swerveModules;
    }

    @Override
    public void zeroSensors() {
        System.out.println("Zeroing drive sensors!");
        resetPigeon();
        setHeading(Rotation2d.identity());

        for (SwerveModule module : swerveModules) {
            if (module != null) {
                module.zeroSensors();
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
        for (SwerveModule module : swerveModules) {
            modulesPassed = modulesPassed && module.checkSystem();
        }

        boolean checkPigeon = mPigeon == null;

        System.out.println(modulesPassed && checkPigeon);
        return modulesPassed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
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
