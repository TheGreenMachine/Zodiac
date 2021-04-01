package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Camera;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class AutoAimAction implements Action {

    private Turret turret;
    private Camera camera;
    private int zone;


    public AutoAimAction(int zone) {
        this();
        this.zone=zone;
    }

    public AutoAimAction() {
        turret = Turret.getInstance();
        camera = Camera.getInstance();
    }

    @Override
    public void start() {
        turret.setZone(zone);
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return camera.getDeltaXAngle() < Camera.ALLOWABLE_AIM_ERROR;
    }

    @Override
    public void done() {}
}
