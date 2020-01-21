package com.team1816.frc2020.controlboard;

import com.team1816.frc2020.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import com.team254.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private final double kDeadband = 0.15;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final Controller mController;

    private GamepadButtonControlBoard() {
        mController = new LogitechController(Constants.kButtonGamepadPort);
        reset();
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public boolean getScorePresetLow() {
        return mController.getButton(LogitechController.Button.A);
    }

    @Override
    public boolean getScorePresetMiddle() {
        return mController.getButton(LogitechController.Button.B);
    }

    @Override
    public boolean getScorePresetHigh() {
        return mController.getButton(LogitechController.Button.Y);
    }

    @Override
    public boolean getScorePresetCargo() {
        return mController.getButton(LogitechController.Button.X);
    }

    @Override
    public boolean getPresetStow() {
        return mController.getButton(LogitechController.Button.LB);
    }

    @Override
    public boolean getPickupDiskWall() {
        return mController.getTrigger(LogitechController.Side.RIGHT);
    }

    @Override
    public boolean getPickupBallGround() {
        return mController.getButton(LogitechController.Button.RB);
    }

    @Override
    public boolean getToggleHangMode() {
        return mController.getButton(LogitechController.Button.START);
    }

    @Override
    public boolean getToggleHangModeLow() {
        return mController.getButton(LogitechController.Button.BACK);
    }

    @Override
    public void reset() {
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    @Override
    public boolean getBeakOpen() {
        return mController.getButton(LogitechController.Button.Y);
    }

    @Override
    public boolean getBeakClose() {
        return mController.getButton(LogitechController.Button.A);
    }

    @Override
    public double getClimberThrottle() {
        return mController.getJoystick(Controller.Side.LEFT, Controller.Axis.Y);
    }

    @Override
    public boolean getShooterOut() {
        return mController.getTrigger(Controller.Side.RIGHT);
    }

    @Override
    public boolean getShooterIn() {
        return mController.getButton(LogitechController.Button.RB);
    }

    @Override
    public boolean getShooterPositionUp() {
        return mController.getButton(LogitechController.Button.B);
    }

    @Override
    public boolean getShooterPositionRocket() {
        return mController.getButton(LogitechController.Button.X);
    }
}
