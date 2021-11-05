package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.Camera;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

import javax.inject.Inject;

public class AutoAimAction implements Action {
    @Inject
    private Turret turret;
    private Camera camera;


    public AutoAimAction() {
        camera = Camera.getInstance();
    }

    @Override
    public void start() {
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
