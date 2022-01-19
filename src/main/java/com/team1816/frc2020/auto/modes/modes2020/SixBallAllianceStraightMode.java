package com.team1816.frc2020.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.frc2020.auto.actions.actions2020.CollectAction;
import com.team1816.frc2020.auto.actions.actions2020.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.actions2020.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class SixBallAllianceStraightMode extends AutoModeBase {

    public DriveTrajectory driveTrajectoryA;
    public DriveTrajectory driveTrajectoryB;
    @Inject
    private static TrajectorySet trajectorySet;

    public SixBallAllianceStraightMode() {
        var trajectoryA = trajectorySet.DRIVE_STRAIGHT_TRENCH;
        var trajectoryB = trajectorySet.DRIVE_STRAIGHT_TRENCH_REVERSE;
        this.driveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        this.driveTrajectoryB = new DriveTrajectory(trajectoryB, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new PrepareToShootAction(0),
                new ShootAction(2.5, false),
                new ParallelAction(new CollectAction(true), driveTrajectoryA),
                new CollectAction(false),
                new ParallelAction(new PrepareToShootAction(15.2), driveTrajectoryB),
                new ShootAction(4, true, 9_300)
            )
        );
    }
}
