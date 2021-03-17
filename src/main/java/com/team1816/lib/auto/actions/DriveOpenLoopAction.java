package com.team1816.lib.auto.actions;

import com.team1816.frc2020.subsystems.Drive;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

public class DriveOpenLoopAction implements Action {

    private static final Drive mDrive = Drive.getInstance();

    private double mStartTime;
    private final double mDuration, mLeft, mRight;

    public DriveOpenLoopAction(double left, double right, double duration) {
        mDuration = duration;
        mLeft = left;
        mRight = right;
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(DriveSignal.fromTank(mLeft, mRight));
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
        mDrive.setOpenLoop(DriveSignal.BRAKE);
    }
}
