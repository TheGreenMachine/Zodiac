package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.LedManager;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class AutoAimAction implements Action {
    private Turret turret;
    private LedManager ledManager;

    public AutoAimAction() {
        turret = Turret.getInstance();
        ledManager = LedManager.getInstance();
    }

    @Override
    public void start() {
        ledManager.setCameraLed(true);
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
