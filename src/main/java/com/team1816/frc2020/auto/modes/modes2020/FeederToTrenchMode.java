package com.team1816.frc2020.auto.modes.modes2020;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class FeederToTrenchMode extends AutoModeBase {

    private DriveTrajectory driveTrajectory;

    public FeederToTrenchMode(boolean reversed) {
        var trajectorySet = TrajectorySet.getInstance();
        var trajectory = reversed
            ? trajectorySet.TRENCH_TO_FEEDER
            : trajectorySet.FEEDER_TO_TRENCH;
        driveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Feeder <-> Trench Mode");
        runAction(new WaitAction(0.5));
        runAction(driveTrajectory);
    }
}
