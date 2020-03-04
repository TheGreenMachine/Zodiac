package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.frc2020.subsystems.LedManager;
import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;

public class ShootAction implements Action {
    private Shooter shooter;
    private Hopper hopper;
    private LedManager ledManager = LedManager.getInstance();
    private AsyncTimer shooterTimer;
    private double turretAngle;
    private Turret turret;

    public ShootAction(double turretAngle) {
        this.shooter = Shooter.getInstance();
        this.hopper = Hopper.getInstance();
        this.turret = Turret.getInstance();
        this.ledManager = LedManager.getInstance();
        this.shooterTimer = new AsyncTimer(4, shooter::startShooter, shooter::stopShooter);
        this.turretAngle = turretAngle;
    }

    @Override
    public void start() {
        // turret.setTurretAngle(turretAngle);
        ledManager.setCameraLed(true);
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
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
        ledManager.setCameraLed(false);
        turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
    }
}
