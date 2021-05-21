package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.auto.actions.actions2020.CollectAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class BlueRedPathBMode extends AutoModeBase {

    private DriveTrajectory trajectory;

    public BlueRedPathBMode() {
        trajectory =
            new DriveTrajectory(TrajectorySet.getInstance().BLUE_RED_PATHB, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Run Blue & Red Path B");
        runAction(
            new SeriesAction(
                new CollectAction(true),
                new WaitAction(0.1),
                trajectory,
                new CollectAction(false)
            )
        );
    }
}
