package com.team1816.frc2020.controlboard;

import com.team1816.frc2020.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.controlboard.LogitechController;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final Controller mController;

    private GamepadDriveControlBoard() {
        mController = new LogitechController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(LogitechController.Side.LEFT, LogitechController.Axis.Y);
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(LogitechController.Side.RIGHT, LogitechController.Axis.X);
    }

    @Override
    public boolean getSlowMode() {
        return mController.getTrigger(LogitechController.Side.RIGHT);
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mController.getButton(LogitechController.Button.Y);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getButton(LogitechController.Button.R_JOYSTICK);
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(LogitechController.Button.LB);
    }

    @Override
    public boolean getCollectorUp() {
        return mController.getButton(LogitechController.Button.RB);
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mController.getButton(LogitechController.Button.X);
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return mController.getButton(LogitechController.Button.B);
    }

    @Override
    public boolean getBrakeMode() {
        return mController.getButton(LogitechController.Button.A);
    }
    @Override
    public boolean getFollowTarget(){return mController.getButton(LogitechController.Button.BACK);}
    @Override
    public int getDriverClimber() {
        switch (mController.getDPad()) {
            case 0:
            case 45:
            case 315:
                return 1;
            case 180:
            case 135:
            case 225:
                return -1;
            default:
                return 0;
        }
    }
}
