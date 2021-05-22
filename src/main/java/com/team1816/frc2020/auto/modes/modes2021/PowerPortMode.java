package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.auto.actions.actions2020.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.actions2020.SetZoneAction;
import com.team1816.frc2020.auto.actions.actions2020.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class PowerPortMode extends AutoModeBase {
    private Trajectory<TimedState<Pose2dWithCurvature>> driveForwardTrajectory;
    private Trajectory<TimedState<Pose2dWithCurvature>> driveReverseTrajectory;

    public PowerPortMode() {
        driveForwardTrajectory = TrajectorySet.getInstance().DRIVE_STRAIGHT_100_FORWARD;
        driveReverseTrajectory = TrajectorySet.getInstance().DRIVE_STRAIGHT_100_REVERSE;
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new SetZoneAction(4),
                new PrepareToShootAction(Turret.CARDINAL_NORTH)
            )
        );
        var resetPose = true;
        for (int i = 0; i < 10; i++) {
            runAction(
                new SeriesAction(
                    new ShootAction(1.2, true),
                    new DriveTrajectory(driveReverseTrajectory, resetPose),
                    new WaitAction(0.6),
                    new ParallelAction(
                        new DriveTrajectory(driveForwardTrajectory, false),
                        new PrepareToShootAction(Turret.CARDINAL_NORTH)
                    )
                )
            );
            resetPose = false;
        }

    }
}
