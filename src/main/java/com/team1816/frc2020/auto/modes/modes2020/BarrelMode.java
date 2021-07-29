package com.team1816.frc2020.auto.modes.modes2020;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;

public class BarrelMode extends AutoModeBase {
    private DriveTrajectory trajectory;

    public BarrelMode() {
        this.trajectory = new DriveTrajectory(TrajectorySet.getInstance().BARREL, true);
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(trajectory);
    }
}
