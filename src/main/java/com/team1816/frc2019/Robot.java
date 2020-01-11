package com.team1816.frc2019;

import badlog.lib.BadLog;
import com.ctre.phoenix.CANifier;
import com.team1816.frc2019.controlboard.ActionManager;
import com.team1816.frc2019.controlboard.ControlBoard;
import com.team1816.frc2019.paths.TrajectorySet;
import com.team1816.frc2019.subsystems.CarriageCanifier;
import com.team1816.frc2019.subsystems.Drive;
import com.team1816.frc2019.subsystems.LedManager;
import com.team1816.frc2019.subsystems.Superstructure;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.lib.subsystems.RobotStateEstimator;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.*;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.wpilib.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

public class Robot extends TimedRobot {
    private BadLog logger;
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // subsystems
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final CarriageCanifier mCarriageCanifer = CarriageCanifier.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final LedManager ledManager = LedManager.getInstance();

    private TimeDelayedBoolean mHangModeEnablePressed = new TimeDelayedBoolean();
    private TimeDelayedBoolean mHangModeLowEnablePressed = new TimeDelayedBoolean();
    private boolean mInHangMode;
    private boolean mIntakeButtonPressed = false;
    private boolean mHangModeReleased = true;

    private MultiTrigger mDiskIntakeTrigger = new MultiTrigger(.4);
    private MultiTrigger mBallIntakeTrigger = new MultiTrigger(.4);

    // button placed on the robot to allow the drive team to zero the robot right
    // before the start of a match
    DigitalInput resetRobotButton = new DigitalInput(Constants.kResetButtonChannel);

    private boolean mHasBeenEnabled = false;
    private double mLastThrustPressedTime = -1.0;
    private double mLastThrustShotTime = Double.NaN;
    private double mLastShootPressedTime = -1.0;
    private double mOffsetOverride = -1.0;

    private LatchedBoolean mShootPressed = new LatchedBoolean();
    private LatchedBoolean mThrustReleased = new LatchedBoolean();
    private LatchedBoolean mThrustPressed = new LatchedBoolean();
    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();
    private LatchedBoolean mAutoSteerPressed = new LatchedBoolean();

    private boolean mStickyShoot;

    private AutoModeSelector mAutoModeSelector = AutoModeSelector.getInstance();
    private AutoModeExecutor mAutoModeExecutor;

    private boolean mDriveByCameraInAuto = false;
    private double loopStart;

    private ActionManager mActionManager;

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    private static RobotFactory factory;

    public static RobotFactory getFactory() {
        if (factory == null) {
            var robotName = System.getenv("ROBOT_NAME");
            if (robotName == null) {
                robotName = "default";
                System.out.println("ROBOT_NAME environment variable not defined, falling back to default.config.yml!");
            }
            factory = new RobotFactory(robotName);
        }
        return factory;
    }

    private Double getLastLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    @Override
    public void robotInit() {
        try {

            var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
            logger = BadLog.init("/home/lvuser/" + System.getenv("ROBOT_NAME") + "_" + logFile + ".bag");
            DrivetrainLogger.init(mDrive);
            BadLog.createValue("PID", String.format("kP = %f, kI = %f, kD = %f, kF = %f", mDrive.getKP(), mDrive.getKI(), mDrive.getKD(), mDrive.getKF()));
            BadLog.createTopic("Timings/Looper", "ms", mEnabledLooper::getLastLoop, "hide", "join:Timings");
            BadLog.createTopic("Timings/RobotLoop", "ms", this::getLastLoop, "hide", "join:Timings");
            BadLog.createTopic("Timings/Timestamp", "s", Timer::getFPGATimestamp, "xaxis", "hide");
            logger.finishInitialization();
            mDrive.setLogger(logger);

            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                mRobotStateEstimator,
                mDrive,
                mSuperstructure,
                mCarriageCanifer,
                mInfrastructure
            );

            mCarriageCanifer.zeroSensors();
            mDrive.zeroSensors();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mInHangMode = false;

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            TrajectorySet.getInstance();

            mAutoModeSelector.updateModeCreator();

            mActionManager = new ActionManager(
                // Driver Gamepad
                //TODO: Setting cargoshooter down or up needs a parallel action that stops intake for both and shooter and collector
                   //      Also needs to raise the collector arm
            );

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mInfrastructure.setIsManualControl(false);

            mDisabledLooper.start();

            mDrive.setBrakeMode(false);
            mThrustReleased.update(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mHasBeenEnabled = true;

            mInfrastructure.setIsManualControl(true); // turn on compressor when superstructure is not moving

            mInHangMode = false;

            mDrive.zeroSensors();

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mHasBeenEnabled = true;

            mEnabledLooper.start();
            mInHangMode = false;

            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();

            mOffsetOverride = -2.0;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private CANifier canifier = Robot.getFactory().getCanifier("ledmanager");

    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();
            double blinkTime = System.currentTimeMillis();

            ledManager.setLedColorBlink(255, 255, 0, 1000);
            while(System.currentTimeMillis() - initTime <= 3000) {
                if (System.currentTimeMillis() - blinkTime > ledManager.getPeriod()) {
                    blinkTime = System.currentTimeMillis();
                    canifier.setLEDOutput((double) (255.0 / 255.0), CANifier.LEDChannel.LEDChannelA);
                    canifier.setLEDOutput((double) (255.0 / 255.0), CANifier.LEDChannel.LEDChannelB);
                    canifier.setLEDOutput((double) (0 / 255.0), CANifier.LEDChannel.LEDChannelC);
                 //   ledManager.setLedColor(ledManager.getLedRgbBlink()[0], ledManager.getLedRgbBlink()[1], ledManager.getLedRgbBlink()[2]);
                } else if (System.currentTimeMillis() - blinkTime > ledManager.getPeriod() / 2) {
                    canifier.setLEDOutput((double) (0 / 255.0), CANifier.LEDChannel.LEDChannelA);
                    canifier.setLEDOutput((double) (0 / 255.0), CANifier.LEDChannel.LEDChannelB);
                    canifier.setLEDOutput((double) (0 / 255.0), CANifier.LEDChannel.LEDChannelC);
                  //  ledManager.setLedColor(0, 0, 0);
                }
            }

            CrashTracker.logTestInit();

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }


    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (!resetRobotButton.get() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!");
                mCarriageCanifer.zeroSensors();
                mDrive.zeroSensors();
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            mCarriageCanifer.writePeriodicOutputs();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        boolean signalToResume = !mControlBoard.getWantsLowGear();
        boolean signalToStop = mControlBoard.getWantsLowGear();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            mAutoModeExecutor.interrupt();
        }

        if (mDriveByCameraInAuto || mAutoModeExecutor.isInterrupted()) {
            manualControl(/*sandstorm=*/true);
        }
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            manualControl(/*sandstorm=*/false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void manualControl(boolean sandstorm) {
        double timestamp = Timer.getFPGATimestamp();
        boolean rumble = false;
        double throttle = mControlBoard.getThrottle();

        Optional<AimingParameters> drive_aim_params = mSuperstructure.getLatestAimingParameters();

        boolean wantShoot = mControlBoard.getShoot();
        boolean shotJustPushed = mShootPressed.update(wantShoot);
        if (shotJustPushed) {
            mLastShootPressedTime = Timer.getFPGATimestamp();
        }

        mActionManager.update();

        boolean wantsLowGear = mControlBoard.getWantsLowGear() && !sandstorm;

        boolean hangModePressed =
            mHangModeEnablePressed.update(mControlBoard.getToggleHangMode(), 0.250);
        boolean hangModeLowPressed =
            mHangModeLowEnablePressed.update(mControlBoard.getToggleHangModeLow(), 0.250);

        if ((hangModeLowPressed && hangModePressed) && !mInHangMode && mHangModeReleased) {
            System.out.println("Entering hang mode for low: " + hangModeLowPressed + " high: " + hangModePressed);
            mInHangMode = true;
            mHangModeReleased = false;
        } else if ((hangModeLowPressed && hangModePressed) && mInHangMode && mHangModeReleased) {
            System.out.println("Exiting hang mode!");
            mInHangMode = false;
            mHangModeReleased = false;
        }

        if (!hangModeLowPressed && !hangModePressed) {
            mHangModeReleased = true;
        }

        // commands
        mDiskIntakeTrigger.update(mControlBoard.getPickupDiskWall());
        mBallIntakeTrigger.update(mControlBoard.getPickupBallGround());
        boolean wants_auto_steer = mControlBoard.getThrust() && mDiskIntakeTrigger.isPressed();
        boolean auto_steer_pressed = mAutoSteerPressed.update(wants_auto_steer);

        if (mInHangMode) {
            mDrive.setHighGear(!wantsLowGear);
        } else {
            // End Effector Jog
            boolean wants_thrust = mControlBoard.getThrust() && !wants_auto_steer;

            boolean thrust_just_pressed = mThrustPressed.update(wants_thrust);

            if (thrust_just_pressed) {
                mLastThrustPressedTime = timestamp;
            }

            if (wants_thrust) {

                if (mStickyShoot) {
                    wantShoot = true;
                    mLastThrustShotTime = timestamp;
                }
            } else {
                mStickyShoot = false;
                final double kLatchShootingTime = 0.75;
                if (!Double.isNaN(mLastThrustShotTime) && timestamp - mLastThrustShotTime < kLatchShootingTime) {
                    wantShoot = true;
                }
            }

            // drive
            mDrive.setHighGear(!wantsLowGear);
            mControlBoard.setRumble(rumble);
        }

        if (wants_auto_steer && !drive_aim_params.isEmpty() && Util.epsilonEquals(drive_aim_params.get().getFieldToVisionTargetNormal().getDegrees(), 0.0, 10.0)) {
            mDrive.autoSteer(Util.limit(throttle, 0.3), drive_aim_params.get());
        } else {
            mDrive.setCheesyishDrive(throttle, -mControlBoard.getTurn(), mControlBoard.getQuickTurn());
        }

    }

    @Override
    public void testPeriodic() {
        ledManager.writePeriodicOutputs();
    }
}

