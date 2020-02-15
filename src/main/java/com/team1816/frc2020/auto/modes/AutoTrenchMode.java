package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
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
        runAction(new WaitAction(.5));
        runAction(mDriveTrajectory);
         //   new SeriesAction(
               // new ShootAction()
//                new ParallelAction(
//                    mDriveTrajectory,
//                    new SeriesAction(
//                        new WaitAction(8),
//                        new CollectorDeployAction
//                    )
//                )
//            )
//        );
    }
}
