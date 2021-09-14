package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.team1816.frc2020.subsystems.Drive;

public class ForceEndPathAction extends RunOnceAction {

    @Inject
    private static Drive.Factory mDriveFactory;
    private Drive mDrive;

    public ForceEndPathAction() {
        mDrive = mDriveFactory.getInstance();
    }

    @Override
    public synchronized void runOnce() {
        mDrive.forceDoneWithPath();
    }


}
