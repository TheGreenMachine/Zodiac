package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.auto.actions.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class SixBallAllianceStraightMode extends AutoModeBase {

    public DriveTrajectory driveTrajectoryA;
    public DriveTrajectory driveTrajectoryB;

    public SixBallAllianceStraightMode() {
        var trajectoryA = TrajectorySet.getInstance().DRIVE_STRAIGHT_TRENCH;
        var trajectoryB = TrajectorySet.getInstance()
            .DRIVE_STRAIGHT_TRENCH_REVERSE;
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
                new ParallelAction(
                    new PrepareToShootAction(15.2),
                    driveTrajectoryB
                ),
                new ShootAction(4, true, 9_300)
            )
        );
    }
}
