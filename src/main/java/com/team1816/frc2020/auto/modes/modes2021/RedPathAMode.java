package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.auto.actions.actions2020.CollectAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class RedPathAMode extends AutoModeBase {

    private DriveTrajectory trajectory;

    public RedPathAMode() {
        trajectory = new DriveTrajectory(TrajectorySet.getInstance().RED_PATHA, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Run Red Path A");
        runAction(
            new SeriesAction(
                new ParallelAction(trajectory, new CollectAction(true)),
                new CollectAction(false)
            )
        );
    }
}
