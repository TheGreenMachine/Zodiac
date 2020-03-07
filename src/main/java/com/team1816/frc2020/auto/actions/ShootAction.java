package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.*;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShootAction implements Action {
    private Shooter shooter;
    private Hopper hopper;
    private LedManager ledManager = LedManager.getInstance();
    private AsyncTimer shooterTimer;
    private Collector collector;
    private Turret turret;

    private boolean unjam;

    private NetworkTableEntry usingVision = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Calibration").getEntry("VISION");


    public ShootAction(boolean unjam) {
        this.shooter = Shooter.getInstance();
        this.hopper = Hopper.getInstance();
        this.turret = Turret.getInstance();
        this.ledManager = LedManager.getInstance();
        this.collector = Collector.getInstance();
        this.shooterTimer = new AsyncTimer(4, shooter::startShooter, shooter::stopShooter);
        this.unjam = unjam;
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
        usingVision.setBoolean(true);
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
        usingVision.setBoolean(false);
    }
}
