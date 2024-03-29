package com.team1816.frc2020.auto.actions.actions2020;

import com.google.inject.Inject;
import com.team1816.frc2020.subsystems.Camera;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;

public class AutoAimAction implements Action {

    @Inject
    private static Turret turret;

    @Inject
    private static Camera camera;

    public AutoAimAction() {
    }

    @Override
    public void start() {
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Math.abs(camera.getDeltaXAngle()) < Camera.ALLOWABLE_AIM_ERROR;
    }

    @Override
    public void done() {}
}
