package com.team1816.frc2019.controlboard;

import com.team1816.frc2019.Constants;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = Constants.kUseDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public void reset() {}

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getWantsLowGear() {
        return mDriveControlBoard.getWantsLowGear();
    }

    @Override
    public boolean getShoot() {
        return mDriveControlBoard.getShoot();
    }

    @Override
    public boolean getThrust() {
        return mDriveControlBoard.getThrust();
    }

    @Override
    public boolean getSlowMode() {
        return mDriveControlBoard.getSlowMode();
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mDriveControlBoard.getDrivetrainFlipped();
    }

    @Override
    public boolean getEjectBeak() {
        return mDriveControlBoard.getEjectBeak();
    }

    @Override
    public boolean getReleaseBeak() {
        return mDriveControlBoard.getReleaseBeak();
    }

    @Override
    public double getCargoIntake() {
        return mDriveControlBoard.getCargoIntake();
    }

    @Override
    public boolean getClimberAndCameraPiston() {
        return mDriveControlBoard.getClimberAndCameraPiston();
    }

    @Override
    public boolean getScorePresetLow() {
        return mButtonControlBoard.getScorePresetLow();
    }

    @Override
    public boolean getScorePresetMiddle() {
        return mButtonControlBoard.getScorePresetMiddle();
    }

    @Override
    public boolean getScorePresetHigh() {
        return mButtonControlBoard.getScorePresetHigh();
    }

    @Override
    public boolean getScorePresetCargo() {
        return mButtonControlBoard.getScorePresetCargo();
    }

    @Override
    public boolean getPresetStow() {
        return mButtonControlBoard.getPresetStow();
    }

    @Override
    public boolean getPickupDiskWall() {
        return mButtonControlBoard.getPickupDiskWall();
    }

    @Override
    public boolean getPickupBallGround() {
        return mButtonControlBoard.getPickupBallGround();
    }

    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getToggleHangMode() {
        return mButtonControlBoard.getToggleHangMode();
    }

    @Override
    public boolean getToggleHangModeLow() {
        return mButtonControlBoard.getToggleHangModeLow();
    }

    @Override
    public boolean getBeakOpen() {
        return mButtonControlBoard.getBeakOpen();
    }

    @Override
    public boolean getBeakClose() {
        return mButtonControlBoard.getBeakClose();
    }

    @Override
    public double getClimberThrottle() {
        return mButtonControlBoard.getClimberThrottle();
    }

    @Override
    public boolean getShooterOut() {
        return mButtonControlBoard.getShooterOut();
    }

    @Override
    public boolean getShooterIn() {
        return mButtonControlBoard.getShooterIn();
    }

    @Override
    public boolean getShooterPositionUp() {
        return mButtonControlBoard.getShooterPositionUp();
    }

    @Override
    public boolean getShooterPositionRocket() {
        return mButtonControlBoard.getShooterPositionRocket();
    }
}
