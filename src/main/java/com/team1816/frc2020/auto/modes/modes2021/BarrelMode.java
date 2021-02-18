package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class BarrelMode extends AutoModeBase {
    private DriveTrajectory trajectory;

    public BarrelMode() {
        trajectory = new DriveTrajectory(
            TrajectorySet.getInstance().BARREL_PATH,
            true
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Barrel Auto Mode");
        runAction(trajectory);
    }
}
