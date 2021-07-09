package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.DistanceManager;
import com.team1816.lib.auto.actions.Action;

public class SetZoneAction implements Action {
    private final DistanceManager distanceManager = DistanceManager.getInstance();
    private final int zone;

    public SetZoneAction(int zone) {
        this.zone = zone;
    }

    @Override
    public void start() {
        distanceManager.setZone(zone);
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
