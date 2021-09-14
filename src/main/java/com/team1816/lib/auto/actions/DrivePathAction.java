package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.google.inject.assistedinject.Assisted;
import com.team1816.frc2020.subsystems.Drive;
import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

    @Inject
    private static Drive.Factory mDriveFactory;
    private PathContainer mPathContainer;
    private Path mPath;
    private Drive mDrive;
    private boolean mStopWhenDone;

    public DrivePathAction(PathContainer p, boolean stopWhenDone) {
        mDrive = mDriveFactory.getInstance();
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        mStopWhenDone = stopWhenDone;
    }

    public DrivePathAction(PathContainer p) {
        //this(p, false);
    }

    @Override
    public void start() {
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void done() {
        if (mStopWhenDone) {
            mDrive.setVelocity(Drive.ZERO_DRIVE_VECTOR);
        }
    }
}
