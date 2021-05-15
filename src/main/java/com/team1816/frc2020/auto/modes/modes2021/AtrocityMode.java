package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;

public class AtrocityMode extends AutoModeBase {

    private DriveTrajectory trajectory;

    public AtrocityMode() {
        trajectory = new DriveTrajectory(TrajectorySet.getInstance().ATROCITY, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Atrocity Auto Mode");
        runAction(trajectory);
    }
}
