package com.team1816.frc2020.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team1816.frc2020.*;
import com.team1816.frc2020.planners.DriveMotionPlanner;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.hardware.TalonSRXChecker;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.TrackableDrivetrain;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.geometry.*;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.ArrayList;

public class Drive extends Subsystem implements TrackableDrivetrain {
    private static Drive mInstance;
    private static final String NAME = "drivetrain";
    private static double DRIVE_ENCODER_PPR;

    private LedManager ledManager = LedManager.getInstance();

    // hardware
    private final IMotorControllerEnhanced mLeftMaster, mRightMaster;
    private final IMotorController mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
    private final Solenoid mShifter;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    // control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;

    // hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private double mLastDriveCurrentSwitchTime = -1;
    private final RobotFactory mFactory = Robot.getFactory();
    private Double kTalonKd;
    private BadLog mLogger;

    private PeriodicIO mPeriodicIO;
    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private Drive() {
        super(NAME);
        DRIVE_ENCODER_PPR = mFactory.getConstant(NAME, "encPPR");
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster = mFactory.getMotor(NAME, "leftMain");
        mLeftSlaveA = mFactory.getMotor(NAME, "leftSlaveOne", mLeftMaster);
        mLeftSlaveB = mFactory.getMotor(NAME, "leftSlaveTwo", mLeftMaster);
        mRightMaster = mFactory.getMotor(NAME, "rightMain");
        mRightSlaveA = mFactory.getMotor(NAME, "rightSlaveOne", mRightMaster);
        mRightSlaveB = mFactory.getMotor(NAME, "rightSlaveTwo", mRightMaster);

        reloadGains();

        mLeftMaster.configOpenloopRamp(Constants.kOpenLoopRampRate, Constants.kCANTimeoutMs);
        mRightMaster.configOpenloopRamp(Constants.kOpenLoopRampRate, Constants.kCANTimeoutMs);

        mShifter = mFactory.getSolenoid("drivetrain", "kShifterSolenoidId");

        if (mFactory.getConstant(NAME, "pigeonOnTalon").intValue() == 1) {
            var pigeonId = mFactory.getConstant(NAME, "pigeonId").intValue();
            System.out.println("Pigeon on Talon " + pigeonId);
            IMotorController master = null;
            if (pigeonId == mLeftSlaveA.getDeviceID()) {
                master = mLeftSlaveA;
            } else if (pigeonId == mLeftSlaveB.getDeviceID()) {
                master = mLeftSlaveB;
            } else if (pigeonId == mRightSlaveA.getDeviceID()) {
                master = mRightSlaveA;
            } else if (pigeonId == mRightSlaveB.getDeviceID()) {
                master = mRightSlaveB;
            }
            if(master != null) {
                mPigeon = new PigeonIMU((TalonSRX) master);
            } else {
                mPigeon = new PigeonIMU(new TalonSRX(mFactory.getConstant(NAME, "pigeonId").intValue()));
            }
        } else {
            mPigeon = new PigeonIMU(mFactory.getConstant(NAME, "pigeonId").intValue());
        }
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10, 10);

        if (mPigeon.getLastError() != ErrorCode.OK) {
            // BadLog.createValue("PigeonErrorDetected", "true");
            System.out.println("Error detected with Pigeon IMU - check if the sensor is present and plugged in!");
            System.out.println("Defaulting to drive straight mode");
            AutoModeSelector.getInstance().setHardwareFailure(true);
        }
        // force a solenoid message
        mIsHighGear = false;
        setHighGear(true);

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = true;
        setBrakeMode(mIsBrakeMode);

        mMotionPlanner = new DriveMotionPlanner();
    }

    private synchronized void reloadGains() {
        reloadTalonGains(mLeftMaster);
        reloadTalonGains(mRightMaster);
    }

    private void reloadTalonGains(IMotorControllerEnhanced talon) {
        kTalonKd = mFactory.getConstant(NAME, "kD");
        talon.config_kP(0, mFactory.getConstant(NAME, "kP"), Constants.kLongCANTimeoutMs);
        talon.config_kI(0, mFactory.getConstant(NAME, "kI"), Constants.kLongCANTimeoutMs);
        talon.config_kD(0, kTalonKd, Constants.kLongCANTimeoutMs);
        talon.config_kF(0, mFactory.getConstant(NAME, "kF"), Constants.kLongCANTimeoutMs);
        talon.config_IntegralZone(0, mFactory.getConstant(NAME, "iZone").intValue(), Constants.kLongCANTimeoutMs);
    }

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public double getHeadingDegrees() {
        return mPeriodicIO.gyro_heading.getDegrees();
    }

    public double getDesiredHeading() {
        if( mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            return mPeriodicIO.path_setpoint.state().getRotation().getDegrees();
        }
        return mPeriodicIO.desired_heading.getDegrees();
    }

    public double getKP() {
        return mFactory.getConstant(NAME, "kP");
    }

    public double getKI() {
        return mFactory.getConstant(NAME, "kI");
    }

    public double getKD() {
        return mFactory.getConstant(NAME, "kD");
    }

    public double getKF() {
        return mFactory.getConstant(NAME, "kF");
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public int left_position_ticks;
        public int right_position_ticks;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        double left_error;
        double right_error;

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public Rotation2d desired_heading = Rotation2d.identity();
        TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(Pose2dWithCurvature.identity());
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);
        mPeriodicIO.left_error = mLeftMaster.getClosedLoopError(0);
        mPeriodicIO.right_error = mRightMaster.getClosedLoopError(0);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        } else {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(true);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case PATH_FOLLOWING:
                            if (mPathFollower != null) {
                                mLogger.updateTopics();
                                mLogger.log();
                                updatePathFollower(timestamp);
                            }
                        case TRAJECTORY_FOLLOWING:
                            mLogger.updateTopics();
                            mLogger.log();
                            updatePathFollower(timestamp);
                            break;

                        default:
                            System.out.println("unexpected drive control state: " + mDriveControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
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
        return inchesToRotations(inches_per_second)* DRIVE_ENCODER_PPR / 10.0;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }

    public synchronized void autoSteer(double throttle, AimingParameters aim_params) {
        double timestamp = Timer.getFPGATimestamp();
        final double kAutosteerAlignmentPointOffset = 15.0;  // Distance from wall
        boolean reverse = throttle < 0.0;
        boolean towards_goal = reverse == (Math.abs(aim_params.getRobotToGoalRotation().getDegrees()) > 90.0);
        Pose2d field_to_vision_target = aim_params.getFieldToGoal();
        final Pose2d vision_target_to_alignment_point = Pose2d.fromTranslation(new Translation2d(Math.min(kAutosteerAlignmentPointOffset, aim_params.getRange() - kAutosteerAlignmentPointOffset), 0.0));
        Pose2d field_to_alignment_point = field_to_vision_target.transformBy(vision_target_to_alignment_point);
        Pose2d vehicle_to_alignment_point = RobotState.getInstance().getFieldToVehicle(timestamp).inverse().transformBy(field_to_alignment_point);
        Rotation2d vehicle_to_alignment_point_bearing = vehicle_to_alignment_point.getTranslation().direction();
        if (reverse) {
            vehicle_to_alignment_point_bearing = vehicle_to_alignment_point_bearing.rotateBy(Rotation2d.fromDegrees(180.0));
        }
        double heading_error_rad = vehicle_to_alignment_point_bearing.getRadians();

        final double kAutosteerKp = 0.05;
        double curvature = (towards_goal ? 1.0 : 0.0) * heading_error_rad * kAutosteerKp;
        setOpenLoop(Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, curvature * throttle * (reverse ? -1.0 : 1.0))));
        setBrakeMode(true);
    }

    /**
     * Configure talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("Switching to Velocity");
            mLeftMaster.selectProfileSlot(0, 0);
            mRightMaster.selectProfileSlot(0, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            // Plumbed default high.
            if( mShifter != null ) mShifter.set(!wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public void setLogger(BadLog logger){
        mLogger = logger;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.desired_heading = heading;
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
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
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
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
        return Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
                new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed,
                    Constants.kMaxLookAheadSpeed),
                Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                Constants.kPathFollowingProfileKs, Constants.kPathFollowingMaxVel,
                Constants.kPathFollowingMaxAccel, Constants.kPathFollowingGoalPosTolerance,
                Constants.kPathFollowingGoalVelTolerance, Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.TRAJECTORY_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }


    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                setVelocity(new DriveSignal(inchesPerSecondToTicksPer100ms(setpoint.getLeft()), inchesPerSecondToTicksPer100ms(setpoint.getRight())), new DriveSignal(0, 0));
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            DriveMotionPlanner.Output output = mMotionPlanner.update(timestamp, RobotState.getInstance().getFieldToVehicle(timestamp));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                    new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        TRAJECTORY_FOLLOWING
    }

    public enum ShifterState {
        FORCE_LOW_GEAR, FORCE_HIGH_GEAR
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {

        setBrakeMode(false);
        setHighGear(true);

//        Timer.delay(3);

        boolean leftSide = TalonSRXChecker.checkMotors(this,
            new ArrayList<>() {
                {
                    add(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster));
                }
            }, getTalonCheckerConfig(mLeftMaster));
        boolean rightSide = TalonSRXChecker.checkMotors(this,
            new ArrayList<>() {
                {
                    add(new TalonSRXChecker.TalonSRXConfig("right_master", mRightMaster));
                }
            }, getTalonCheckerConfig(mRightMaster));
        boolean checkPigeon = mPigeon == null;

        System.out.println(leftSide && rightSide && checkPigeon);
        if (leftSide && rightSide && checkPigeon){
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        }
        else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return leftSide && rightSide;
    }

    private TalonSRXChecker.CheckerConfig getTalonCheckerConfig(IMotorControllerEnhanced talon) {
        return new TalonSRXChecker.CheckerConfig() {
            {
                mCurrentFloor = Robot.getFactory().getConstant(NAME,"currentFloorCheck");
                mRPMFloor = Robot.getFactory().getConstant(NAME,"rpmFloorCheck");
                mCurrentEpsilon = Robot.getFactory().getConstant(NAME,"currentEpsilonCheck");
                mRPMEpsilon = Robot.getFactory().getConstant(NAME,"rpmEpsilonCheck");
                mRPMSupplier = () -> talon.getSelectedSensorVelocity(0);
            }
        };
    }

    @Override
    public double getLeftVelocityDemand() {
        return mPeriodicIO.left_demand;
    }

    @Override
    public double getRightVelocityDemand() {
        return mPeriodicIO.right_demand;
    }

    @Override
    public double getLeftVelocityError() {
        return mPeriodicIO.left_error;
    }

    @Override
    public double getRightVelocityError() {
        return mPeriodicIO.right_error;
    }

    public double getRightDriveTicks() { return mPeriodicIO.right_position_ticks; }

    public double getLeftDriveTicks() { return mPeriodicIO.left_position_ticks; }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Right Drive Distance", this::getRightEncoderDistance, null);
        builder.addDoubleProperty("Right Drive Ticks", this::getRightDriveTicks, null);
        builder.addDoubleProperty("Left Drive Distance", this::getLeftEncoderDistance, null);
        builder.addDoubleProperty("Left Drive Ticks", this::getLeftDriveTicks, null);
        // SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        // SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        // SmartDashboard.putNumber("X Error", mPeriodicIO.error.getTranslation().x());
        // SmartDashboard.putNumber("Y error", mPeriodicIO.error.getTranslation().y());
        // SmartDashboard.putNumber("Theta Error", mPeriodicIO.error.getRotation().getDegrees());

        // SmartDashboard.putNumber("Left Voltage Kf", mPeriodicIO.left_voltage / getLeftLinearVelocity());
        // SmartDashboard.putNumber("Right Voltage Kf", mPeriodicIO.right_voltage / getRightLinearVelocity());

        // if (mPathFollower != null) {
        //     SmartDashboard.putNumber("Drive LTE", mPathFollower.getAlongTrackError());
        //     SmartDashboard.putNumber("Drive CTE", mPathFollower.getCrossTrackError());
        // } else {
        //     SmartDashboard.putNumber("Drive LTE", 0.0);
        //     SmartDashboard.putNumber("Drive CTE", 0.0);
        // }

        if (getHeading() != null) {
            builder.addDoubleProperty("Gyro Heading", this::getHeadingDegrees, null);
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}
