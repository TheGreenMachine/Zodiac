package com.team1816.frc2020.controlboard;

import com.team1816.frc2020.Constants;
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
    public boolean getSlowMode() {
        return mDriveControlBoard.getSlowMode();
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mDriveControlBoard.getDrivetrainFlipped();
    }

    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return mDriveControlBoard.getSpinnerThreeTimes();
    }

    @Override
    public boolean getSpinnerColor() {
        return mDriveControlBoard.getSpinnerColor();
    }

    @Override
    public boolean getSpinnerReset() {
        return mButtonControlBoard.getSpinnerReset();
    }
}
