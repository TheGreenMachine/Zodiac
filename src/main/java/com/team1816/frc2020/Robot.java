package com.team1816.frc2020;

import badlog.lib.BadLog;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.frc2020.controlboard.ActionManager;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.*;
import com.team1816.lib.LibModule;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.lib.subsystems.RobotStateEstimator;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.SwerveDriveSignal;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

import static com.team1816.frc2020.controlboard.ControlUtils.*;

public class Robot extends TimedRobot {

    private BadLog logger;

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard;

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // subsystems
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive;
    private final LedManager ledManager = LedManager.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Turret turret = Turret.getInstance();
    // private final Spinner spinner = Spinner.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Camera camera = Camera.getInstance();
    private final RobotStateEstimator mRobotStateEstimator;

    private boolean mHasBeenEnabled = false;

    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private AutoModeSelector mAutoModeSelector = AutoModeSelector.getInstance();
    private AutoModeExecutor mAutoModeExecutor;

    private boolean mDriveByCameraInAuto = false;
    private double loopStart;

    private ActionManager actionManager;
    private AsyncTimer blinkTimer;

    // private PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Turret.ControlMode prevTurretControlMode = Turret.ControlMode.FIELD_FOLLOWING;

    Robot() {
        super();
        // initialize injector
        Injector injector = Guice.createInjector(new LibModule(), new SeasonModule());
        mDrive = (injector.getInstance(Drive.Factory.class)).getInstance();
        mControlBoard = injector.getInstance(IControlBoard.class);
        mRobotStateEstimator = injector.getInstance(RobotStateEstimator.class);
    }

    public static RobotFactory getFactory() {
        return RobotFactory.getInstance();
    }

    private Double getLastLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    @Override
    public void robotInit() {
        try {
            DriverStation.getInstance().silenceJoystickConnectionWarning(true);
            var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
            var robotName = System.getenv("ROBOT_NAME");
            if (robotName == null) robotName = "default";
            var logFileDir = "/home/lvuser/";
            // if there is a usb drive use it
            if (Files.exists(Path.of("/media/sda1"))) {
                logFileDir = "/media/sda1/";
            }
            if (RobotBase.isSimulation()) {
                if (System.getProperty("os.name").toLowerCase().contains("win")) {
                    logFileDir = System.getenv("temp") + "\\";
                } else {
                    logFileDir = System.getProperty("user.dir") + "/";
                }
            }
            var filePath = logFileDir + robotName + "_" + logFile + ".bag";
            logger = BadLog.init(filePath);

            BadLog.createValue(
                "Max Velocity",
                String.valueOf(Constants.kPathFollowingMaxVel)
            );
            BadLog.createValue(
                "Max Acceleration",
                String.valueOf(Constants.kPathFollowingMaxAccel)
            );


            BadLog.createTopic(
                "Collector/TargetVel",
                "NativeUnits",
                collector::getIntakePow,
                "hide",
                "join:Collector/Velocities"
            );

            BadLog.createTopic(
                "Collector/ActVel",
                "NativeUnits",
                collector::getActualVelocity,
                "hide",
                "join:Collector/Velocities"
            );

            if (Constants.kIsBadlogEnabled) {
                BadLog.createTopic(
                    "Timings/Looper",
                    "ms",
                    mEnabledLooper::getLastLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/RobotLoop",
                    "ms",
                    this::getLastLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/Timestamp",
                    "s",
                    Timer::getFPGATimestamp,
                    "xaxis",
                    "hide"
                );

                // BadLog.createTopic("PDP/Current", "Amps", pdp::getTotalCurrent);





                BadLog.createValue("Drivetrain PID", mDrive.pidToString());
                BadLog.createValue("Shooter PID", shooter.pidToString());
                BadLog.createValue("Turret PID", turret.pidToString());

                BadLog.createTopic(
                    "Vision/DeltaXAngle",
                    "Degrees",
                    camera::getDeltaXAngle
                );
                BadLog.createTopic("Vision/Distance", "inches", camera::getDistance);
                BadLog.createTopic("Vision/CenterX", "pixels", camera::getRawCenterX);

                BadLog.createTopic(
                    "Turret/ActPos",
                    "NativeUnits",
                    turret::getTurretPositionTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                BadLog.createTopic(
                    "Turret/TargetPos",
                    "NativeUnits",
                    turret::getTargetPosition,
                    "hide",
                    "join:Turret/Positions"
                );
                BadLog.createTopic(
                    "Turret/ErrorPos",
                    "NativeUnits",
                    turret::getPositionError
                );
            }

            logger.finishInitialization();

            mSubsystemManager.setSubsystems(
                mRobotStateEstimator,
                mDrive,
                mSuperstructure,
                mInfrastructure,
                shooter,
                // spinner,
                collector,
                hopper,
                turret,
                climber
            );

            mDrive.zeroSensors();
            turret.zeroSensors();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            ledManager.registerEnabledLoops(mEnabledLooper);
            ledManager.registerEnabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(
                Timer.getFPGATimestamp(),
                Pose2d.identity(),
                Rotation2d.identity()
            );
            mDrive.setHeading(Rotation2d.identity());

            TrajectorySet.getInstance();

            mAutoModeSelector.updateModeCreator();

            actionManager =
                new ActionManager();
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();

            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);

            // shooter

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
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

            // Robot starts forwards.
            mRobotState.reset(
                Timer.getFPGATimestamp(),
                Pose2d.identity(),
                Rotation2d.identity()
            );
            mDrive.setHeading(Rotation2d.identity());

            mHasBeenEnabled = true;

            mInfrastructure.setIsManualControl(true); // turn on compressor when superstructure is not moving

            mDrive.setOpenLoop(SwerveDriveSignal.NEUTRAL);

            mDrive.zeroSensors();
            turret.zeroSensors();

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);

            turret.zeroSensors();



            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mHasBeenEnabled = true;

            mEnabledLooper.start();

            turret.setTurretAngle(Turret.CARDINAL_SOUTH);
            turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);

            mDrive.setOpenLoop(SwerveDriveSignal.NEUTRAL);

            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();

            ledManager.blinkStatus(LedManager.RobotStatus.DRIVETRAIN_FLIPPED);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writePeriodicOutputs();
            }

            mEnabledLooper.stop();
            mDisabledLooper.start();

            blinkTimer.reset();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
            mRobotStateEstimator.outputToSmartDashboard();
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (RobotController.getUserButton() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!");
                mDrive.zeroSensors();
                turret.zeroSensors();
                mRobotState.reset(
                    Timer.getFPGATimestamp(),
                    Pose2d.identity(),
                    Rotation2d.identity()
                );
                mDrive.setHeading(Rotation2d.identity());
                ledManager.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
            } else {
                ledManager.indicateDefaultStatus();
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (
                autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()
            ) {
                System.out.println(
                    "Set auto mode to: " + autoMode.get().getClass().toString()
                );
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        boolean signalToResume = !mControlBoard.getDrivetrainFlipped(); // TODO: select auto interrupt button
        boolean signalToStop = mControlBoard.getDrivetrainFlipped();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            System.out.println("Auto mode interrupted ");
            mAutoModeExecutor.interrupt();
        }

        if (mDriveByCameraInAuto || mAutoModeExecutor.isInterrupted()) {
            manualControl();
        }

        if (Constants.kIsLoggingAutonomous) {
            logger.updateTopics();
            logger.log();
        }
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl();
        } catch (Throwable t) {
            throw t;
        }

        if (Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
        }
    }

    TimeDelayedBoolean mShouldMaintainAzimuth = new TimeDelayedBoolean();
    LatchedBoolean shouldChangeAzimuthSetpoint = new LatchedBoolean();

    public void manualControl() {
        // boolean arcadeDrive = false;
        actionManager.update();

        boolean maintainAzimuth = mShouldMaintainAzimuth.update(
            mControlBoard.getTurn() == 0,
            0.2
        );
        boolean changeAzimuthSetpoint = shouldChangeAzimuthSetpoint.update(
            maintainAzimuth
        );

//        if (mControlBoard.getDPad() != -1) {
//            swerveHeadingController.setState(
//                SwerveHeadingController.State.Snap
//            );
//            double heading_goal = mControlBoard.getDPad();
//            SmartDashboard.putNumber("Heading Goal", heading_goal);
//            swerveHeadingController.setGoal(heading_goal);
//        } else {
//            if (!maintainAzimuth) {
//                swerveHeadingController.setState(
//                    SwerveHeadingController.State.Off
//                );
//            } else if (
//                (
//                    swerveHeadingController.getState() ==
//                    SwerveHeadingController.State.Snap &&
//                    swerveHeadingController.is()
//                ) ||
//                changeAzimuthSetpoint
//            ) {
//                swerveHeadingController.setState(
//                    SwerveHeadingController.HeadingControllerState.MAINTAIN
//                );
//                swerveHeadingController.setGoal(mDrive.getHeading().getDegrees());
//            }
//        }
//
//        if (
//            swerveHeadingController.getHeadingControllerState() !=
//            SwerveHeadingController.HeadingControllerState.OFF
//        ) {
//            mDrive.setTeleopInputs(
//                mControlBoard.getThrottle(),
//                mControlBoard.getStrafe(),
//                swerveHeadingController.update(),
//                mControlBoard.getSlowMode(),
//                mControlBoard.getFieldRelative(),
//                true
//            );
//        } else {
            mDrive.setTeleopInputs(
                mControlBoard.getThrottle(),
                mControlBoard.getStrafe(),
                mControlBoard.getTurn(),
                mControlBoard.getSlowMode(),
                /*mControlBoard.getFieldRelative()*/ // Field Relative override button conflicts with collector
                false
            );
//        }
    }

    @Override
    public void testPeriodic() {}
}
