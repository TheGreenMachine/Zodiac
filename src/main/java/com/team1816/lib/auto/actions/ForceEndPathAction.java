package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.team1816.frc2020.subsystems.Drive;

public class ForceEndPathAction extends RunOnceAction {

    private Drive mDrive;

    @Inject
    public ForceEndPathAction(Drive.Factory driveFactory) {
        mDrive = driveFactory.getInstance();
    }

    @Override
    public synchronized void runOnce() {
        mDrive.forceDoneWithPath();
    }


}
