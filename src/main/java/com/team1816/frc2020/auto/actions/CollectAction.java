package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Collector;
import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.lib.auto.actions.Action;

public class CollectAction implements Action {
    private boolean isCollecting;

    public CollectAction(boolean isCollecting) {
        this.isCollecting = isCollecting;
    }

    @Override
    public void start() {
        System.out.println("Modifying collector!");
        Collector.getInstance().setDeployed(isCollecting);
        Hopper.getInstance().setSpindexer(isCollecting ? 1 : 0);
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
