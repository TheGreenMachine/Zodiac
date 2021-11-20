package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.Camera;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.Action;
import javax.inject.Inject;

public class AutoAimAction implements Action {


    private Turret turret;

    private Camera camera;

    public AutoAimAction(Turret turret) {
        this.turret = turret;
        camera = Camera.getInstance();
    }

    @Override
    public void start() {
        turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
        System.out.println("Starting Aim Action");
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        System.out.println("Delta: "+camera.getDeltaXAngle());
        System.out.println(Math.abs(camera.getDeltaXAngle()) <
            Camera.ALLOWABLE_AIM_ERROR);
        return Math.abs(camera.getDeltaXAngle()) <
            Camera.ALLOWABLE_AIM_ERROR;
    }

    @Override
    public void done() {}
}
