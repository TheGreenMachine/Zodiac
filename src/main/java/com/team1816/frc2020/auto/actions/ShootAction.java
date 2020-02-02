package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;

public class ShootAction implements Action {

    private Shooter shooter;
    private AsyncTimer shooterTimer;

    public ShootAction() {
        this.shooter = Shooter.getInstance();
        this.shooterTimer = new AsyncTimer(3000, shooter::startShooter, shooter::stopShooter);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        shooterTimer.update();
    }

    @Override
    public boolean isFinished() {
        return shooterTimer.isCompleted();
    }

    @Override
    public void done() {
        shooter.stopShooter();
    }
}
