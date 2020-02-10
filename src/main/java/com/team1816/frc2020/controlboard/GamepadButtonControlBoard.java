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
    public void reset() {
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    @Override
    public boolean getSpinnerReset() {
        return mController.getDPad() == 0;
    }

    @Override
    public boolean getFeederFlapperOut() {
        return mController.getButton(LogitechController.Button.Y);
    }

    @Override
    public boolean getFeederFlapperIn() {
        return mController.getButton(LogitechController.Button.A);
    }
}
