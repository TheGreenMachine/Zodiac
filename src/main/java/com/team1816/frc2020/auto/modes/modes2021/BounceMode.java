package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class BounceMode extends AutoModeBase {
    private DriveTrajectory trajectory1;
    private DriveTrajectory trajectory2;

    public BounceMode() {
        trajectory1 = new DriveTrajectory(TrajectorySet.getInstance().BOUNCE_PATH_1, true);
 //       trajectory2 = new DriveTrajectory(TrajectorySet.getInstance().BOUNCE_PATH_2_F, true);

        trajectory2 = new DriveTrajectory(TrajectorySet.getInstance().BOUNCE_PATH_2_F, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                trajectory1,
       //         new WaitAction(1),
                trajectory2
            )
        );
    }
}
