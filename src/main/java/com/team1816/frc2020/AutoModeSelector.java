package com.team1816.frc2020;

import com.team1816.frc2020.auto.modes.modes2020.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.auto.modes.DoNothingMode;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AutoModeSelector {

    private boolean hardwareFailure = false;

    enum StartingPosition {
        LEFT_HAB_2,
        RIGHT_HAB_2,
        CENTER_HAB_1,
        LEFT_HAB_1,
        RIGHT_HAB_1,
    }

    enum DesiredMode {
        // 2020
        DRIVE_BY_CAMERA,
        DO_NOTHING,
        TUNE_DRIVETRAIN,
        TUNE_DRIVETRAIN_REVERSE,
        TURRET_TEST,
        LIVING_ROOM,
        DRIVE_STRAIGHT,
        AUTO_TRENCH_TURN_RIGHT,
        FIVE_BALL_OPPOSING,
        SIX_BALL_ALLIANCE,
        EIGHT_BALL_ALLIANCE,
        EIGHT_BALL_ALLIANCE_ALT,
        EIGHT_BALL_OPPOSE,
        TEN_BALL_AUTO,
        DRIVE_STRAIGHT_SHOOT,
        SIX_BALL_ALLIANCE_STRAIGHT,

        // 2021
        BARREL
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();

        mStartPositionChooser.setDefaultOption(
            "Center HAB 1",
            StartingPosition.CENTER_HAB_1
        );

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();

        // 2020

        mModeChooser.addOption("Drive By Camera", DesiredMode.DRIVE_BY_CAMERA);
        mModeChooser.addOption("Tune Drivetrain", DesiredMode.TUNE_DRIVETRAIN);
        mModeChooser.addOption(
            "Tune Drivetrain Reverse",
            DesiredMode.TUNE_DRIVETRAIN_REVERSE
        );
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        SmartDashboard.putData("Auto mode", mModeChooser);

        // CheezeCurd
        mModeChooser.addOption("Living Room", DesiredMode.LIVING_ROOM);
        //        mModeChooser.addOption("Shop", DesiredMode.SHOP);
        //        mModeChooser.addOption("PID", DesiredMode.PID);
        mModeChooser.setDefaultOption("Drive Straight", DesiredMode.DRIVE_STRAIGHT);
        mModeChooser.addOption("Turret Tuning", DesiredMode.TURRET_TEST);
        mModeChooser.addOption(
            "Auto Trench Turn Right",
            DesiredMode.AUTO_TRENCH_TURN_RIGHT
        );
        mModeChooser.addOption(
            "Auto Trench Turn Right",
            DesiredMode.AUTO_TRENCH_TURN_RIGHT
        );

        mModeChooser.addOption("Drive Straight Shoot", DesiredMode.DRIVE_STRAIGHT_SHOOT);

        // ALLIANCE
        mModeChooser.addOption("6 Ball Alliance Trench", DesiredMode.SIX_BALL_ALLIANCE);
        mModeChooser.addOption(
            "6 Ball Alliance Straight",
            DesiredMode.SIX_BALL_ALLIANCE_STRAIGHT
        );
        mModeChooser.addOption("8 Ball Alliance Trench", DesiredMode.EIGHT_BALL_ALLIANCE);
        mModeChooser.addOption(
            "8 Ball Alliance Trench ALT (NOT TESTED)",
            DesiredMode.EIGHT_BALL_ALLIANCE_ALT
        );

        // OPPOSING
        mModeChooser.addOption("5 Ball Opposing Trench", DesiredMode.FIVE_BALL_OPPOSING);
        mModeChooser.addOption(
            "8 Ball Opposing Trench (NOT TESTED)",
            DesiredMode.EIGHT_BALL_OPPOSE
        );

        mModeChooser.addOption(
            "10 Ball Trench (Not yet implemented DO NOT USE)",
            DesiredMode.TEN_BALL_AUTO
        );

        mModeChooser.addOption("Barrel", DesiredMode.BARREL);


        SmartDashboard.putData("Auto mode", mModeChooser);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);
    }

    public void setHardwareFailure(boolean hasFailed) {
        if (RobotBase.isReal()) {
            hardwareFailure = hasFailed;
        }
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if (
            mCachedDesiredMode != desiredMode ||
            startingPosition != mCachedStartingPosition
        ) {
            System.out.println(
                "Auto selection changed, updating creator: desiredMode->" +
                desiredMode.name() +
                ", starting position->" +
                startingPosition.name()
            );
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return (
            position == StartingPosition.LEFT_HAB_1 ||
            position == StartingPosition.LEFT_HAB_2
        );
    }

    private boolean startingHab1(StartingPosition position) {
        return (
            position == StartingPosition.LEFT_HAB_1 ||
            position == StartingPosition.RIGHT_HAB_1
        );
    }

    private Optional<AutoModeBase> getAutoModeForParams(
        DesiredMode mode,
        StartingPosition position
    ) {
        if (hardwareFailure) {
            return Optional.of(new DriveStraightMode());
        }
        switch (mode) {
            // 2020
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case DRIVE_BY_CAMERA:
                return Optional.of(new DriveByCameraMode());
            case TUNE_DRIVETRAIN:
                return Optional.of(new TuneDrivetrainMode(false));
            case TUNE_DRIVETRAIN_REVERSE:
                return Optional.of(new TuneDrivetrainMode(true));
            case TURRET_TEST:
                return Optional.of(new TurretTestMode());
            case DRIVE_STRAIGHT:
                return (Optional.of(new DriveStraightMode()));
            case LIVING_ROOM:
                return (Optional.of(new LivingRoomMode()));
            case SIX_BALL_ALLIANCE:
                return (Optional.of(new SixBallAllianceMode()));
            case FIVE_BALL_OPPOSING:
                return (Optional.of(new FiveBallOpposingTrenchMode()));
            case EIGHT_BALL_ALLIANCE:
                return (Optional.of(new EightBallAllianceMode()));
            case EIGHT_BALL_ALLIANCE_ALT:
                return (Optional.of(new EightBallAllianceAltMode()));
            case EIGHT_BALL_OPPOSE:
                return (Optional.of(new EightBallOpposeMode()));
            case TEN_BALL_AUTO:
                return (Optional.of(new TenBallMode()));
            case DRIVE_STRAIGHT_SHOOT:
                return (Optional.of(new DriveStraightShootMode()));
            case SIX_BALL_ALLIANCE_STRAIGHT:
                return (Optional.of(new SixBallAllianceStraightMode()));
            case BARREL:
                return (Optional.of(new BarrelMode()));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString(
            "StartingPositionSelected",
            mCachedStartingPosition.name()
        );
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DRIVE_BY_CAMERA;
    }

    private static AutoModeSelector INSTANCE;

    public static AutoModeSelector getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AutoModeSelector();
        }
        return INSTANCE;
    }
}
