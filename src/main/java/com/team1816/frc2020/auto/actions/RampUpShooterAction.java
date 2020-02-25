package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class RampUpShooterAction implements Action {
    @Override
    public void start() {
        Shooter.getInstance().startShooter();
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
