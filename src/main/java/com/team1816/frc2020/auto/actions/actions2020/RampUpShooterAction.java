package com.team1816.frc2020.auto.actions.actions2020;

import com.google.inject.Inject;
import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.lib.auto.actions.Action;

public class RampUpShooterAction implements Action {

    @Inject
    private static Shooter shooter;

    @Override
    public void start() {
        shooter.startShooter();
        shooter.autoHood();
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
