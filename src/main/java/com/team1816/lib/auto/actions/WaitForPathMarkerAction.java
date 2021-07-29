package com.team1816.lib.auto.actions;

import com.team1816.frc2020.subsystems.SwerveDrive;

public class WaitForPathMarkerAction implements Action {

    private SwerveDrive mDrive = SwerveDrive.getInstance();
    private String mMarker;

    public WaitForPathMarkerAction(String marker) {
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
