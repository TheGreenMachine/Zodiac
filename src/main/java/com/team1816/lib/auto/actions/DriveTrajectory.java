package com.team1816.lib.auto.actions;

import com.google.inject.assistedinject.Assisted;
import com.team1816.frc2020.RobotState;
import com.team1816.frc2020.subsystems.Drive;
import com.team1816.frc2020.subsystems.SwerveDrive;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;

import javax.inject.Inject;

public class DriveTrajectory implements Action {

    private static Drive mDrive;
    private static final RobotState mRobotState = RobotState.getInstance();

    private TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private Rotation2d targetHeading;
    private boolean mResetPose;
    private boolean done;

    @Inject
    public DriveTrajectory(
        Drive.Factory driveFactory,
        @Assisted Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
        @Assisted Rotation2d targetHeading,
        @Assisted boolean resetPose
    ) {
        mDrive = driveFactory.getInstance();
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        this.targetHeading = targetHeading;
        mResetPose = resetPose;
    }

    @Inject
    public DriveTrajectory(
        Drive.Factory driveFactory,
        @Assisted Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
        @Assisted boolean resetPose
    ) {
        this(driveFactory, trajectory, Rotation2d.identity(), resetPose);

    }

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            if (!done) {
                System.out.println("Trajectory finished");
            }
            done = true;
            return true;
        }
        return false;
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {
        System.out.println(
            "Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")"
        );
        var pose = mTrajectory.getState().state().getPose();

        if (mResetPose) {
            mDrive.setHeading(pose.getRotation());
            mRobotState.reset(Timer.getFPGATimestamp(), pose);
            if(mDrive instanceof SwerveDrive) {
                ((SwerveDrive)mDrive).setStartingPose(pose);
                ((SwerveDrive)mDrive).setWantReset(true);
            }

        }
        mDrive.setTrajectory(mTrajectory, targetHeading);
    }
}
