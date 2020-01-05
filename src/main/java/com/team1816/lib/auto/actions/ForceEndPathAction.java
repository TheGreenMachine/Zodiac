package com.team1816.lib.auto.actions;

import com.team1816.frc2019.subsystems.Drive;

public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() {
        Drive.getInstance().forceDoneWithPath();
    }
}