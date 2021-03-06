package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;

public class UTurnMode extends AutoModeBase {

    DriveTrajectory trajectory;

    public UTurnMode() {
        trajectory  = new DriveTrajectory(TrajectorySet.getInstance().U_TURN_PATH, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running U Turn Path");
        runAction (
            trajectory
        );
    }

}
