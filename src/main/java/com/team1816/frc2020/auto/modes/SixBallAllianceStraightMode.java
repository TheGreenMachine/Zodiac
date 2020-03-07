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
    public DriveTrajectory driveTrajectory;

    public SixBallAllianceStraightMode() {
        var trajectory = TrajectorySet.getInstance().DRIVE_STRAIGHT_TRENCH;
        this.driveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new PrepareToShootAction(0),
                new ShootAction( false),
                new ParallelAction(
                    new CollectAction(true),
                    new PrepareToShootAction(15.2),
                    driveTrajectory
                ),
                new CollectAction(false),
                new ShootAction(true)
            )
        );
    }
}
