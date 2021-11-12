package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;
import javax.inject.Inject;

public class TurretAction implements Action {

    @Inject
    private Turret turret;

    private double turretAngle;

    public TurretAction(double turretAngle) {
        this.turretAngle = turretAngle;
    }

    @Override
    public void start() {
        turret.setTurretAngle(turretAngle);
        // turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return turret.getPositionError() < 10;
    }

    @Override
    public void done() {}
}
