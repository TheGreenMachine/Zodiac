package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class TurretAction implements Action {
    private final Turret turret = Turret.getInstance();
    private double turretAngle;

    public TurretAction(double turretAngle) {
        this.turretAngle = turretAngle;
    }

    @Override
    public void start() {
        turret.setTurretAngle(turretAngle);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return turret.getPositionError() < 10;
    }

    @Override
    public void done() {

    }
}
