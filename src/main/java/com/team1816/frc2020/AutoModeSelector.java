package com.team1816.frc2020;

import com.team1816.frc2020.auto.AutoModes;
import com.team1816.frc2020.auto.modes.modes2020.DriveStraightMode;
import com.team1816.lib.auto.modes.AutoModeBase;
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

    private AutoModes mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<AutoModes> mModeChooser;
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

        for (AutoModes mode : AutoModes.values()) {
            mModeChooser.addOption(mode.getName(), mode);
        }

        SmartDashboard.putData("Starting Position", mStartPositionChooser);
    }

    public void setHardwareFailure(boolean hasFailed) {
        hardwareFailure = hasFailed;
    }

    public void updateModeCreator() {
        var desiredMode = mModeChooser.getSelected();
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
        AutoModes mode,
        StartingPosition position
    ) {
        if (hardwareFailure) {
            return Optional.of(new DriveStraightMode());
        }
        return Optional.of(mode.getConstructor().get());
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
        return false;
    }

    private static AutoModeSelector INSTANCE;

    public static AutoModeSelector getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AutoModeSelector();
        }
        return INSTANCE;
    }
}
