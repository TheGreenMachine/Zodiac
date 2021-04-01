package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.lib.auto.actions.Action;

public class AdjustHoodAction implements Action {

    private Shooter shooter;
    private boolean rodIn;

    public AdjustHoodAction(boolean rodIn){
        shooter=Shooter.getInstance();
        this.rodIn=!rodIn;

    }

    @Override
    public void start() {
        shooter.setHoodRod(rodIn);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
