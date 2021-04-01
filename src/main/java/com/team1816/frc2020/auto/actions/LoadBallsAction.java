package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Collector;
import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.lib.auto.actions.Action;

public class LoadBallsAction implements Action {
    private Collector collector;
    private Hopper hopper;
    private Boolean PreLoad;
    public LoadBallsAction(boolean PreLoad){
        this.PreLoad=PreLoad;
        collector=Collector.getInstance();
        hopper=Hopper.getInstance();
    }
    @Override
    public void start() {
        collector.setIntakePow(-0.5);
        hopper.setIntake(0.6);

    }

    @Override
    public void update() {
        if(isFinished()){
            collector.setIntakePow(0);
            hopper.setIntake(0);
        }
    }

    @Override
    public boolean isFinished() {
        if (!PreLoad) {
            return true;
        }
        return !hopper.getHasBall();
    }

    @Override
    public void done() {
        collector.setIntakePow(0);
        hopper.setIntake(0);

    }
}
