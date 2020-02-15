package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.HopperFeedAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class AutoTrenchMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectory;

    public AutoTrenchMode() {
        var trajectory = TrajectorySet.getInstance().AUTO_TRENCH;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Auto Trench Mode");
        runAction(
            new SeriesAction(
//                new ParallelAction(
//                    new HopperFeedAction(4000),
//                    new SeriesAction(
//                        new WaitAction(1),
//                        new ShootAction()
//                    )
//                ),
//                new WaitAction(.5),
                mDriveTrajectory
            )
        );
    }
}
