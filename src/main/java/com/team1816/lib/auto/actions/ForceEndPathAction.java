package com.team1816.lib.auto.actions;

import com.team1816.frc2020.subsystems.SwerveDrive;

public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() {
        SwerveDrive.getInstance().forceDoneWithPath();
    }
}
