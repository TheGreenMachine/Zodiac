package com.team1816.frc2020.auto.actions.actions2020;

import com.google.inject.Inject;
import com.team1816.frc2020.subsystems.*;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;

public class ShootAction implements Action {

    @Inject
    private static Turret turret;
    @Inject
    private static Shooter shooter;
    @Inject
    private static Hopper hopper;
    @Inject
    private static LedManager ledManager;
    @Inject
    private static Collector collector;

    private AsyncTimer shooterTimer;

    private boolean unjam;

    public ShootAction(double duration, boolean unjam, double velocity) {
        this(duration, unjam);
        this.shooterTimer =
            new AsyncTimer(
                duration,
                () -> shooter.setVelocity(velocity),
                shooter::stopShooter
            );
    }

    public ShootAction(double duration, boolean unjam) {
        this.shooterTimer =
            new AsyncTimer(duration, shooter::startShooter, shooter::stopShooter);
        this.unjam = unjam;
    }

    public ShootAction(boolean unjam) {
        this(4, unjam);
    }

    @Override
    public void start() {
        // turret.setTurretAngle(turretAngle);
        ledManager.setCameraLed(true);
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
        shooterTimer.update();
        collector.setIntakePow(0.5);
        hopper.lockToShooter(true, unjam);
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
        hopper.lockToShooter(false, true);
        hopper.setIntake(0);
        collector.setIntakePow(0);
        ledManager.setCameraLed(false);
        turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
    }
}
