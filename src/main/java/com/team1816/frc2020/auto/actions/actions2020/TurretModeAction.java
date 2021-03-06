package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class TurretModeAction implements Action {

    private final Turret turret = Turret.getInstance();
    private Turret.ControlMode mode;

    public TurretModeAction(Turret.ControlMode mode) {
        this.mode = mode;
    }

    @Override
    public void start() {
         turret.setControlMode(mode);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
