package com.team1816.frc2020.controlboard;

import com.team1816.frc2020.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import com.team1816.lib.controlboard.XboxController;

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
        mController = new XboxController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        System.out.println("Left Joystick: "+ mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X));
        return mController.getJoystick(
            XboxController.Side.LEFT,
            XboxController.Axis.Y
        );
    }

    @Override
    public double getTurn() {
        System.out.println("Right Joystick: "+ mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X));
        return mController.getJoystick(
            XboxController.Side.RIGHT,
            XboxController.Axis.X
        );
    }

    @Override
    public boolean getSlowMode() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mController.getButton(XboxController.Button.Y);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getButton(XboxController.Button.R_JOYSTICK);
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getCollectorUp() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mController.getButton(XboxController.Button.X);
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public boolean getBrakeMode() {
        return mController.getButton(XboxController.Button.A);
    }

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
