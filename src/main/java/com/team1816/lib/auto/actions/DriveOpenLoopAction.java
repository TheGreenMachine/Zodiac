package com.team1816.lib.auto.actions;

import com.google.inject.assistedinject.Assisted;
import com.team1816.frc2020.subsystems.Drive;
import com.team254.lib.util.SwerveDriveSignal;
import edu.wpi.first.wpilibj.Timer;

import javax.inject.Inject;

public class DriveOpenLoopAction implements Action {

    private static Drive mDrive;

    private double mStartTime;
    private final double mDuration, mLeft, mRight;

    @Inject
    public DriveOpenLoopAction(Drive.Factory driveFactory, @Assisted double left, @Assisted double right, @Assisted double duration) {
        mDrive = driveFactory.getInstance();
        mDuration = duration;
        mLeft = left;
        mRight = right;
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new SwerveDriveSignal(mLeft, mRight));
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(SwerveDriveSignal.BRAKE);
    }
}
