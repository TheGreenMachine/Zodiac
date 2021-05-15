package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.auto.actions.actions2020.CollectAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class GoodSnowRemovalMode extends AutoModeBase {
    private DriveTrajectory trajectory1;
    private DriveTrajectory trajectory2;
    private DriveTrajectory trajectory3;

    public GoodSnowRemovalMode() {
        trajectory1 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_REMOVAL_1, true);
        trajectory2 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_REMOVAL_2, false);
        trajectory3 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_REMOVAL_3, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new ParallelAction(trajectory1, new CollectAction(true)),
                new ParallelAction(trajectory2, new CollectAction(true)),
                new CollectAction(false),
                trajectory3
            )
        );
    }
}
