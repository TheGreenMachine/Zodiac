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
    public boolean getCollectorDown() {
        return mDriveControlBoard.getCollectorDown();
    }

    @Override
    public boolean getCollectorUp() {
        return mDriveControlBoard.getCollectorUp();
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mDriveControlBoard.getFeederToTrenchSpline();
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return mDriveControlBoard.getTrenchToFeederSpline();
    }

    @Override
    public boolean getBrakeMode() {
        return false;
    }

    @Override
    public int getDriverClimber() {
        return mDriveControlBoard.getDriverClimber();
    }


    // Button Control Board
    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return mButtonControlBoard.getSpinnerThreeTimes();
    }

    @Override
    public boolean getTurretJogLeft() {
        return mButtonControlBoard.getTurretJogLeft();
    }

    @Override
    public boolean getTurretJogRight() {
        return mButtonControlBoard.getTurretJogRight();
    }

    @Override
    public boolean getAutoHome() {
        return mButtonControlBoard.getAutoHome();
    }

    @Override
    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    @Override
    public boolean getSpinnerColor() {
        return mButtonControlBoard.getSpinnerColor();
    }

    @Override
    public boolean getSpinnerReset() {
        return mButtonControlBoard.getSpinnerReset();
    }

    @Override
    public boolean getFeederFlapOut() {
        return mButtonControlBoard.getFeederFlapOut();
    }

    @Override
    public boolean getFeederFlapIn() {
        return mButtonControlBoard.getFeederFlapIn();
    }

    @Override
    public double getClimber() {
        return mButtonControlBoard.getClimber();
    }

    @Override
    public boolean getClimberDeploy() {
        return mButtonControlBoard.getClimberDeploy();
    }
}
