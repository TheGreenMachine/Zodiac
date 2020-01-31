package com.team1816.frc2019.controlboard;

import com.team1816.frc2019.Constants;
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
    public boolean getWantsLowGear() {
        return false;
    }

    @Override
    public boolean getThrust() {
        return false;
    }

    @Override
    public boolean getSlowMode() {
        return mController.getButton(LogitechController.Button.RB);
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mController.getButton(LogitechController.Button.Y);
    }

    @Override
    public boolean getEjectBeak() {
        return mController.getButton(LogitechController.Button.LB);
    }

    @Override
    public boolean getReleaseBeak() {
        return mController.getTrigger(Controller.Side.LEFT);
    }

    @Override
    public double getCargoIntake() {
        return -1 * mController.getTriggerScalar(Controller.Side.RIGHT);
    }

    @Override
    public boolean getClimberAndCameraPiston() {
        return mController.getButton(LogitechController.Button.B);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getButton(LogitechController.Button.R_JOYSTICK);
    }

    @Override
    public boolean getShoot() {
        return false;
    }
}
