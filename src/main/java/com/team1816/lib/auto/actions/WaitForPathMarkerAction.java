package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.google.inject.assistedinject.Assisted;
import com.team1816.frc2020.subsystems.Drive;

public class WaitForPathMarkerAction implements Action {

    @Inject
    private static Drive.Factory mDriveFactory;
    private Drive mDrive;
    private String mMarker;

    public WaitForPathMarkerAction( String marker) {
        mDrive = mDriveFactory.getInstance();
        mMarker = marker;
    }

    @Override
    public boolean isFinished() {
        return mDrive.hasPassedMarker(mMarker);
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {}
}
