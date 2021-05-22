package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.Collector;
import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.lib.auto.actions.Action;

public class LoadBallsAction implements Action {
    private final Collector collector;
    private final Hopper hopper;

    public LoadBallsAction() {
        this.collector = Collector.getInstance();
        this.hopper = Hopper.getInstance();
    }

    @Override
    public void start() {
        collector.setIntakePow(-1);
        hopper.setIntake(0.5);
    }

    @Override
    public void update() {
        if (isFinished()) {
            collector.setIntakePow(0);
            hopper.setIntake(0);
        }
    }

    @Override
    public boolean isFinished() {
        return hopper.hasBall();
    }

    @Override
    public void done() {
        collector.setIntakePow(0);
        hopper.setIntake(0);
    }
}
