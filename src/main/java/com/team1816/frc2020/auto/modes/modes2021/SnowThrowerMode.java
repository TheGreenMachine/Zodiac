package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class SnowThrowerMode extends AutoModeBase {
    private DriveTrajectory trajectory1;
    private DriveTrajectory trajectory2;
    private DriveTrajectory trajectory3;

    public SnowThrowerMode() {
        trajectory1 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_THROWER_1, true);
        trajectory2 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_THROWER_2, false);
        trajectory3 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_THROWER_3, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                trajectory1,
                trajectory2,
                trajectory3
            )
        );
    }
}
