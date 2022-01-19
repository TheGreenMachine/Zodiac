package com.team1816.frc2020.auto.actions.actions2020;

import com.google.inject.Inject;
import com.team1816.frc2020.subsystems.Collector;
import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.lib.auto.actions.Action;

public class CollectAction implements Action {

    private boolean isCollecting;

    @Inject
    private static Collector collector;
    @Inject
    private static Hopper hopper;

    public CollectAction(boolean isCollecting) {
        this.isCollecting = isCollecting;
    }

    @Override
    public void start() {
        System.out.println("Modifying collector!");
        collector.setDeployed(isCollecting, false);
        hopper.setSpindexer(isCollecting ? -1 : 0);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
