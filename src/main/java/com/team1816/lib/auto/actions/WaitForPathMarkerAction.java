package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.google.inject.assistedinject.Assisted;
import com.team1816.frc2020.subsystems.Drive;

public class WaitForPathMarkerAction implements Action {

    private Drive mDrive;
    private String mMarker;

    @Inject
    public WaitForPathMarkerAction(Drive.Factory driveFactory, @Assisted String marker) {
        mDrive = driveFactory.getInstance();
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
