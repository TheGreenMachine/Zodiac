package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.lib.auto.actions.Action;

public class RampUpShooterAction implements Action {
    private int zone;
    public RampUpShooterAction(int zone){
        this.zone=zone;
    }
    public RampUpShooterAction(){
        super();
    }

    @Override
    public void start() {
        Shooter.getInstance().setZone(zone);
        Shooter.getInstance().startShooter();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Shooter.getInstance().getTargetVelocity()-Shooter.getInstance().getActualVelocity()<100;
    }

    @Override
    public void done() {}
}
