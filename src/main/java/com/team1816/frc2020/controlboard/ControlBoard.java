package com.team1816.frc2020.controlboard;

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
        mDriveControlBoard =  GamepadDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public void reset() {}

    // Drive Control Board
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
    public boolean getSpinnerColor() {
        return mDriveControlBoard.getSpinnerColor();
    }


    @Override
    public boolean getCollectorDown() {
        return mDriveControlBoard.getCollectorDown();
    }

    @Override
    public boolean getCollectorUp() {
        return mDriveControlBoard.getCollectorUp();
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return mDriveControlBoard.getSpinnerThreeTimes();
    }


    // Button Control Board
    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getSpinnerReset() {
        return mButtonControlBoard.getSpinnerReset();
    }

    @Override
    public boolean getFeederFlapperOut() {
        return mButtonControlBoard.getFeederFlapperOut();
    }

    @Override
    public boolean getFeederFlapperIn() {
        return mButtonControlBoard.getFeederFlapperIn();
    }

    @Override
    public int getClimber() {
        return mButtonControlBoard.getClimber();
    }
}
