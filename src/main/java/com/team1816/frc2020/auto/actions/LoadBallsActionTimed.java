package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Collector;
import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.lib.auto.actions.Action;

public class LoadBallsActionTimed implements Action {

    private Collector collector;
    private Hopper hopper;
    private long startTime;
    private long duration;

    public LoadBallsActionTimed(Long duration){
        collector=Collector.getInstance();
        hopper=Hopper.getInstance();
        this.duration=duration;
    }
    @Override
    public void start() {

        collector.setIntakePow(-1);
        hopper.setIntake(0.7);
        startTime = System.currentTimeMillis();

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
        return (System.currentTimeMillis()-startTime)<duration;
    }

    @Override
    public void done() {
        collector.setIntakePow(0);
        hopper.setIntake(0);

    }
}
