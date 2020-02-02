package com.team1816.frc2020.auto.actions;

import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;

public class HopperFeedAction implements Action {

    private Hopper hopper;
    private AsyncTimer hopperTimer;

    public HopperFeedAction(int duration) {
        this.hopper = Hopper.getInstance();
        this.hopperTimer = new AsyncTimer(duration,
            () -> {
                hopper.setSpindexer(1);
                hopper.setElevator(1);
            },
            () -> {
                hopper.setSpindexer(0);
                hopper.setElevator(0);
            }
        );
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        hopperTimer.update();
    }

    @Override
    public boolean isFinished() {
        return hopperTimer.isCompleted();
    }

    @Override
    public void done() {
        hopper.setSpindexer(0);
    }
}
