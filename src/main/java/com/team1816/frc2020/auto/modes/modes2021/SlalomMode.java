package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;

public class SlalomMode extends AutoModeBase {
    private DriveTrajectory trajectory;

    public SlalomMode() {
        trajectory = new DriveTrajectory(TrajectorySet.getInstance().SLALOM_PATH, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Slalom Auto Mode");
        runAction(trajectory);
    }
}
