package com.team1816.frc2020;

import static com.team1816.frc2020.controlboard.ControlUtils.*;

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
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.lib.subsystems.RobotStateEstimator;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.SwerveDriveSignal;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

public class Robot extends TimedRobot {

    private BadLog logger;

    private final Injector injector;

    private final Looper mEnabledLooper = new Looper(this);
    private final Looper mDisabledLooper = new Looper(this);

    private IControlBoard mControlBoard;

    private final SubsystemManager mSubsystemManager;

    // subsystems
    private final Superstructure mSuperstructure;
    private final Infrastructure mInfrastructure;
    private final RobotState mRobotState;
    private final Drive mDrive;
    private final PowerDistribution pdp = new PowerDistribution();
    private final LedManager ledManager;
    private final Collector collector;
    private final Shooter shooter;
    private final Turret mTurret;
    // private final Spinner spinner = Spinner.getInstance();
    private final Hopper hopper;
    private final Climber climber;
    private final Camera camera;
    private final RobotStateEstimator mRobotStateEstimator;

    private boolean mHasBeenEnabled = false;

    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private AutoModeSelector mAutoModeSelector;
    private AutoModeExecutor mAutoModeExecutor;
    private TrajectorySet trajectorySet;

    private boolean mDriveByCameraInAuto = false;
    private double loopStart;

    private ActionManager actionManager;
    private AsyncTimer blinkTimer;

    // private PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Turret.ControlMode prevTurretControlMode = Turret.ControlMode.FIELD_FOLLOWING;

    Robot() {
        super(Constants.kLooperDt);
        // initialize injector
        injector = Guice.createInjector(new LibModule(), new SeasonModule());
        mDrive = (injector.getInstance(Drive.Factory.class)).getInstance();
        mRobotStateEstimator = injector.getInstance(RobotStateEstimator.class);
        mTurret = injector.getInstance(Turret.class);
        mRobotState = injector.getInstance(RobotState.class);
        collector = injector.getInstance(Collector.class);
        mSuperstructure = injector.getInstance(Superstructure.class);
        mInfrastructure = injector.getInstance(Infrastructure.class);
        ledManager = injector.getInstance(LedManager.class);
        shooter = injector.getInstance(Shooter.class);
        hopper = injector.getInstance(Hopper.class);
        climber = injector.getInstance(Climber.class);
        camera = injector.getInstance(Camera.class);
        mSubsystemManager = injector.getInstance(SubsystemManager.class);
        mAutoModeSelector = injector.getInstance(AutoModeSelector.class);
        trajectorySet = injector.getInstance(TrajectorySet.class);
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
            mControlBoard = injector.getInstance(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);
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

            shooter.CreateBadLogTopic(
                "Shooter/ActVel",
                "NativeUnits",
                shooter::getActualVelocity,
                "hide",
                "join:Shooter/Velocities"
            );
            shooter.CreateBadLogTopic(
                "Shooter/TargetVel",
                "NativeUnits",
                shooter::getTargetVelocity,
                "hide",
                "join:Shooter/Velocities"
            );
            shooter.CreateBadLogTopic(
                "Shooter/Error",
                "NativeUnits",
                shooter::getError,
                "hide",
                "join:Shooter/Velocities"
            );

            collector.CreateBadLogTopic(
                "Collector/TargetVel",
                "NativeUnits",
                collector::getIntakePow,
                "hide",
                "join:Collector/Velocities"
            );

            collector.CreateBadLogTopic(
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

                BadLog.createTopic("PDP/Current", "Amps", pdp::getTotalCurrent);

                DrivetrainLogger.init(mDrive);
                mDrive.CreateBadLogValue("Drivetrain PID", mDrive.pidToString());
                shooter.CreateBadLogValue("Shooter PID", shooter.pidToString());
                mTurret.CreateBadLogValue("Turret PID", mTurret.pidToString());

                BadLog.createTopic(
                    "Vision/DeltaXAngle",
                    "Degrees",
                    camera::getDeltaXAngle
                );
                BadLog.createTopic("Vision/Distance", "inches", camera::getDistance);
                BadLog.createTopic("Vision/CenterX", "pixels", camera::getRawCenterX);

                mTurret.CreateBadLogTopic(
                    "Turret/ActPos",
                    "NativeUnits",
                    mTurret::getActualTurretPositionTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/TargetPos",
                    "NativeUnits",
                    mTurret::getTargetPosition,
                    "hide",
                    "join:Turret/Positions"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/ErrorPos",
                    "NativeUnits",
                    mTurret::getPositionError
                );
                mTurret.CreateBadLogTopic(
                    "Turret/FieldToTurret",
                    "Degrees",
                    mRobotState::getLatestFieldToTurret,
                    "hide",
                    "join:Tracking/Angles"
                );
                mTurret.CreateBadLogTopic(
                    "Drive/HeadingRelativeToInitial",
                    "Degrees",
                    () -> mDrive.getHeadingRelativeToInitial().getDegrees(),
                    "hide",
                    "join:Tracking/Angles"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/TurretAngle",
                    "Degrees",
                    mTurret::getActualTurretPositionDegrees,
                    "hide",
                    "join:Tracking/Angles"
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
                mTurret,
                climber
            );

            mDrive.zeroSensors();
            mTurret.zeroSensors();

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

            mAutoModeSelector.updateModeCreator();

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    createHoldAction(
                        mControlBoard::getCollectorToggle,
                        collecting -> {
                            System.out.println("Collector toggled!");
                            collector.setDeployed(collecting, false);
                            hopper.setSpindexer(collecting ? -1 : 0);
                        }
                    ),
                    createScalar(
                        mControlBoard::getDriverClimber,
                        climber::setClimberPower
                    ),
                    createAction(
                        mControlBoard::getClimberDeploy,
                        () -> {
                            if (
                                (DriverStation.getInstance().getMatchTime() <= 30) ||
                                (DriverStation.getInstance().getMatchTime() == -1)
                            ) {
                                climber.setDeployed(!climber.getDeployed());
                            }
                        }
                    ),
                    createAction(
                        mControlBoard::getTrenchToFeederSpline,
                        () -> {
                            System.out.println("STARTING TRENCH TO FEEDER");
                            SmartDashboard.putString("Teleop Spline", "TRENCH TO FEEDER");
                            var trajectory = new DriveTrajectory(
                                trajectorySet.TRENCH_TO_FEEDER,
                                true
                            );
                            trajectory.start();
                        }
                    ),
                    createAction(
                        mControlBoard::getFeederToTrenchSpline,
                        () -> {
                            System.out.println("STARTING FEEDER TO TRENCH");
                            SmartDashboard.putString("Teleop Spline", "FEEDER TO TRENCH");
                            mTurret.setTurretAngle(Turret.CARDINAL_SOUTH);
                            var trajectory = new DriveTrajectory(
                                trajectorySet.FEEDER_TO_TRENCH,
                                true
                            );
                            trajectory.start();
                        }
                    ),
                    createHoldAction(mControlBoard::getSlowMode, mDrive::setSlowMode),
                    // Operator Gamepad
                    // createAction(mControlBoard::getSpinnerReset, spinner::initialize),
                    // createHoldAction(mControlBoard::getSpinnerColor, spinner::goToColor),
                    // createHoldAction(
                    //     mControlBoard::getSpinnerThreeTimes,
                    //     spinner::spinThreeTimes
                    // ),
                    createAction(
                        mControlBoard::getFieldFollowing,
                        () -> mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING)
                    ),
                    createHoldAction(
                        mControlBoard::getFeederFlapOut,
                        feeding -> {
//                            if (mDrive.hasPigeonResetOccurred() && !feeding) {
//                                mDrive.zeroSensors();
//                            }
                            hopper.setFeederFlap(feeding);
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getClimberUp,
                        pressed -> {
                            if (
                                (DriverStation.getInstance().getMatchTime() <= 30) ||
                                (DriverStation.getInstance().getMatchTime() == -1)
                            ) {
                                climber.setDeployed(true);
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getClimberDown,
                        pressed -> {
                            if (
                                (DriverStation.getInstance().getMatchTime() <= 30) ||
                                (DriverStation.getInstance().getMatchTime() == -1)
                            ) {
                                climber.setClimberPower(pressed ? 0.7 : 0);
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogLeft,
                        moving ->
                            mTurret.setTurretSpeed(moving ? -Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogRight,
                        moving ->
                            mTurret.setTurretSpeed(moving ? Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getAutoAim,
                        pressed -> {
                            if (pressed) {
                                prevTurretControlMode = mTurret.getControlMode();
                                mTurret.setControlMode(
                                    Turret.ControlMode.CAMERA_FOLLOWING
                                );
                                shooter.startShooter();
                            } else {
                                mTurret.setControlMode(prevTurretControlMode);
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getShoot,
                        shooting -> {
                            if (shooting) {
                                mDrive.setOpenLoop(SwerveDriveSignal.BRAKE);
                                shooter.startShooter(); // Uses ZED distance
                                mTurret.lockTurret();
                            } else {
                                mTurret.setControlMode(
                                    Turret.ControlMode.FIELD_FOLLOWING
                                );
                                shooter.stopShooter();
                                shooter.setHood(false);
                            }
                            hopper.lockToShooter(shooting, false);
                            hopper.setIntake(shooting ? 1 : 0);
                            collector.setIntakePow(shooting ? 0.5 : 0);
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getCollectorBackspin,
                        pressed -> {
                            collector.setDeployed(pressed, true);
                            hopper.setSpindexer(pressed ? -1 : 0);
                        }
                    ),
                    createAction(
                        mControlBoard::getFeederFlapIn,
                        () -> shooter.setHood(!shooter.isHoodOut())
                    )
                );


            climber.setDeployed(false);

            blinkTimer =
                new AsyncTimer(
                    3, // (3 s)
                    () -> ledManager.blinkStatus(LedManager.RobotStatus.ERROR),
                    () -> ledManager.indicateStatus(LedManager.RobotStatus.OFF)
                );
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
            shooter.setVelocity(0);

            climber.setDeployed(false);

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
            mTurret.zeroSensors();

            mTurret.setTurretAngle(Turret.CARDINAL_SOUTH);
            mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);

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

            mTurret.zeroSensors();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mHasBeenEnabled = true;

            mEnabledLooper.start();
            mTurret.setTurretAngle(Turret.CARDINAL_SOUTH);
            mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
            System.out.println(mTurret.getActualTurretPositionTicks() + "+++++++"); // for debugging whether or not getActTicks works. doesn't seem to - ginget

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
                mTurret.zeroSensors();
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
            /*mControlBoard.getFieldRelative()*/// Field Relative override button conflicts with collector
            false
        );
        //        }
    }

    @Override
    public void testPeriodic() {}
}
