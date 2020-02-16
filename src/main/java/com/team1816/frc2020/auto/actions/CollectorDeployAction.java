package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Collector;
import com.team1816.lib.auto.actions.Action;

public class CollectorDeployAction implements Action {
    private boolean isDeployed;

    public CollectorDeployAction(boolean isDeployed) {
        this.isDeployed = isDeployed;
    }

    @Override
    public void start() {
        Collector.getInstance().setDeployed(isDeployed);
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
