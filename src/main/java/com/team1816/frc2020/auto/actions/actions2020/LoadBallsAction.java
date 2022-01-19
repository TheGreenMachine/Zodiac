package com.team1816.frc2020.auto.actions.actions2020;

import com.google.inject.Inject;
import com.team1816.frc2020.subsystems.Collector;
import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.lib.auto.actions.Action;

public class LoadBallsAction implements Action {

    @Inject
    private static Collector collector;
    @Inject
    private static Hopper hopper;

    public LoadBallsAction() {
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
