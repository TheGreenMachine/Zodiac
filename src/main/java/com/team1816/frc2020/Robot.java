package com.team1816.frc2020;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import com.team1816.frc2020.controlboard.ActionManager;
import com.team1816.frc2020.controlboard.ControlBoard;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.*;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.actions.DriveTrajectory;
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
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.LatchedBoolean;
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

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // subsystems
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final LedManager ledManager = LedManager.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Turret turret = Turret.getInstance();
    // private final Spinner spinner = Spinner.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Camera camera = Camera.getInstance();

    // button placed on the robot to allow the drive team to zero the robot right
    // before the start of a match
    DigitalInput resetRobotButton = new DigitalInput(Constants.kResetButtonChannel);

    private boolean mHasBeenEnabled = false;

    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private AutoModeSelector mAutoModeSelector = AutoModeSelector.getInstance();
    private AutoModeExecutor mAutoModeExecutor;

    private boolean mDriveByCameraInAuto = false;
    private double loopStart;
    private boolean faulted;

    private ActionManager actionManager;
    private CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

    private PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Turret.ControlMode prevTurretControlMode = Turret.ControlMode.FIELD_FOLLOWING;

    Robot() {
        super();
    }

    private static RobotFactory factory;

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
            if (System.getProperty("os.name").toLowerCase().contains("win")) {
                logFileDir = System.getenv("temp") + "\\";
            } else if (System.getProperty("os.name").toLowerCase().contains("mac")) {
                logFileDir = System.getProperty("user.dir") + "/";
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
                "Shooter/ActVel",
                "NativeUnits",
                shooter::getActualVelocity,
                "hide",
                "join:Shooter/Velocities"
            );
            BadLog.createTopic(
                "Shooter/TargetVel",
                "NativeUnits",
                shooter::getTargetVelocity,
                "hide",
                "join:Shooter/Velocities"
            );
            BadLog.createTopic(
                "Shooter/Error",
                "NativeUnits",
                shooter::getError,
                "hide",
                "join:Shooter/Velocities"
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

                BadLog.createTopic("PDP/Current", "Amps", pdp::getTotalCurrent);

                BadLog.createTopicSubscriber(
                    "Pigeon Error",
                    BadLog.UNITLESS,
                    DataInferMode.DEFAULT
                );

                DrivetrainLogger.init(mDrive);

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
                    () -> (double) turret.getTurretPositionTicks(),
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

//                BadLog.createTopic(
//                    "Turret/FieldToTurret",
//                    "Degrees",
//                    mRobotState::getLatestFieldToTurret,
//                    "hide",
//                    "join:Tracking/Angles"
//                );
                BadLog.createTopic(
                    "Drive/HeadingRelativeToInitial",
                    "Degrees",
                    () -> mDrive.getHeadingRelativeToInitial().getDegrees(),
                    "hide",
                    "join:Tracking/Angles"
                );
                BadLog.createTopic(
                    "Turret/TurretAngle",
                    "Degrees",
                    turret::getActualTurretPositionDegrees,
                    "hide",
                    "join:Tracking/Angles"
                );

                mDrive.setLogger(logger);
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
                new ActionManager(
                    // Driver Gamepad
                    createHoldAction(
                        mControlBoard::getCollectorToggle,
                        collecting -> {
                            System.out.println("Collector toggled!");
                            collector.setDeployed(collecting);
                            hopper.setSpindexer(collecting ? -1 : 0);
                        }
                    ),
                    createScalar(
                        mControlBoard::getDriverClimber,
                        climber::setClimberPower
                    ),
                    createHoldAction(
                        mControlBoard::getClimberDeploy,
                        pressed -> {
                            if (
                                (DriverStation.getInstance().getMatchTime() <= 30) ||
                                (DriverStation.getInstance().getMatchTime() == -1)
                            ) {
                                climber.setDeployed(pressed);
                            }
                        }
                    ),
                    createAction(
                        mControlBoard::getTrenchToFeederSpline,
                        () -> {
                            System.out.println("STARTING TRENCH TO FEEDER");
                            SmartDashboard.putString("Teleop Spline", "TRENCH TO FEEDER");
                            var trajectory = new DriveTrajectory(
                                TrajectorySet.getInstance().TRENCH_TO_FEEDER,
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
                            turret.setTurretAngle(Turret.CARDINAL_SOUTH);
                            var trajectory = new DriveTrajectory(
                                TrajectorySet.getInstance().FEEDER_TO_TRENCH,
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
                        () -> turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING)
                    ),
                    createHoldAction(
                        mControlBoard::getFeederFlapOut,
                        feeding -> {
                            if (mDrive.hasPigeonResetOccurred() && !feeding) {
                                mDrive.zeroSensors();
                            }
                            hopper.setFeederFlap(feeding);
                        }
                    ),
                    createScalar(
                        mControlBoard::getClimber,
                        power -> {
                            if (
                                (DriverStation.getInstance().getMatchTime() <= 30) ||
                                (DriverStation.getInstance().getMatchTime() == -1)
                            ) {
                                climber.setClimberPower(power > 0 ? power : 0);
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogLeft,
                        moving ->
                            turret.setTurretSpeed(moving ? -Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogRight,
                        moving ->
                            turret.setTurretSpeed(moving ? Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getAutoAim,
                        pressed -> {
                            if (pressed) {
                                prevTurretControlMode = turret.getControlMode();
                                turret.setControlMode(
                                    Turret.ControlMode.CAMERA_FOLLOWING
                                );
                                shooter.startShooter();
                            } else {
                                turret.setControlMode(prevTurretControlMode);
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getShoot,
                        shooting -> {
                            // shooter.setVelocity(shooting ? Shooter.MID_VELOCITY : 0);
                            if (shooting) {
                                mDrive.setOpenLoop(DriveSignal.BRAKE);
                                shooter.startShooter(); // Uses ZED distance
                                turret.lockTurret();
                            } else {
                                turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
                                shooter.stopShooter();
                            }
                            hopper.lockToShooter(shooting, true);
                            hopper.setIntake(shooting ? 1 : 0);
                            collector.setIntakePow(shooting ? 0.5 : 0);
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getCollectorBackSpin,
                        pressed -> collector.setIntakePow(pressed ? 0.2 : 0)
                    )
                );

        } catch (Throwable t) {
            faulted = true;
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
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            if(faulted) {
                DriverStation.reportError("Robot is in a FAULTED STATE Not Executing Auto !!!!!!", false);
                return;
            }
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

            mDrive.zeroSensors();
            turret.zeroSensors();

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            if(faulted) {
                DriverStation.reportError("Robot is in a FAULTED STATE Not Executing Teleop !!!!!!", false);
                return;
            }
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

            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            if(faulted) {
                DriverStation.reportError("Robot is in a FAULTED STATE Not Executing Test !!!!!!", false);
                return;
            }
            double initTime = System.currentTimeMillis();

            ledManager.blinkStatus(LedManager.RobotStatus.DRIVETRAIN_FLIPPED);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writePeriodicOutputs();
            }

            mEnabledLooper.stop();
            mDisabledLooper.start();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            faulted = true;
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
            faulted = true;
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
                if (faulted) {
                    ledManager.blinkStatus(LedManager.RobotStatus.ERROR);
                } else {
                    ledManager.indicateStatus(LedManager.RobotStatus.DISABLED);
                }
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
            faulted = true;
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
            faulted = true;
            throw t;
        }

        if (Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
        }
    }

    public void manualControl() {
        // boolean arcadeDrive = false;
        actionManager.update();

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        DriveSignal driveSignal;

        // if (arcadeDrive) {
        //            var filteredThrottle = Math.signum(throttle) * (throttle * throttle);
        //            double left = Util.limit(filteredThrottle + (turn * 0.55), 1);
        //            double right = Util.limit(filteredThrottle - (turn * 0.55), 1);
        //            driveSignal = new DriveSignal(left, right);
        // } else {
        driveSignal = cheesyDriveHelper.cheesyDrive(throttle, turn, false); // quick turn temporarily eliminated
        // }
        if (
            mDrive.getDriveControlState() == Drive.DriveControlState.TRAJECTORY_FOLLOWING
        ) {
            if (
                driveSignal.getLeft() != 0 ||
                driveSignal.getRight() != 0 ||
                mDrive.isDoneWithTrajectory()
            ) {
                mDrive.setOpenLoop(driveSignal);
            }
        } else if (!(shooter.getTargetVelocity() > 0)) {
            mDrive.setOpenLoop(driveSignal);
        } else {
            mDrive.setOpenLoop(DriveSignal.BRAKE);
        }
    }

    @Override
    public void testPeriodic() {}
}
