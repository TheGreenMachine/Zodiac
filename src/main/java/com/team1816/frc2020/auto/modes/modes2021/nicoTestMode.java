package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class nicoTestMode extends AutoModeBase {

    private DriveTrajectory trajectory1;
    private DriveTrajectory trajectory2;
    private DriveTrajectory trajectory3;


    public nicoTestMode() {
        trajectory1 = new DriveTrajectory(TrajectorySet.getInstance().NICO_TEST1, true);
        trajectory2 = new DriveTrajectory(TrajectorySet.getInstance().NICO_TEST2, false);
        trajectory3 = new DriveTrajectory(TrajectorySet.getInstance().NICO_TEST3, false);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running nico test Auto Mode");
        runAction(
            new SeriesAction(
                trajectory1,
                trajectory2,
                trajectory3
            )
        );
    }
}
