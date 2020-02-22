package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class RampUpShooterAction implements Action {
    private double turretAngle;

    public RampUpShooterAction(double turretAngle) {
        this.turretAngle = turretAngle;
    }

    @Override
    public void start() {
        Shooter.getInstance().startShooter();
        Turret.getInstance().setTurretAngle(turretAngle);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Turret.getInstance().getPositionError() < 10;
    }

    @Override
    public void done() {

    }
}
