package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;

public class ShootAction implements Action {

    private Shooter shooter;
    private Hopper hopper;
    private AsyncTimer shooterTimer;

    public ShootAction() {
        this.shooter = Shooter.getInstance();
        this.hopper = Hopper.getInstance();
        this.shooterTimer = new AsyncTimer(9, shooter::startShooter, shooter::stopShooter);
    }

    @Override
    public void start() {
        Turret.getInstance().setTurretAngle(Turret.CARDINAL_WEST);
        shooterTimer.update();
        hopper.lockToShooter(true);
        hopper.setIntake(1);
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
        hopper.lockToShooter(false);
        hopper.setIntake(0);
    }
}
