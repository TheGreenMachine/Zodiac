package com.team1816.frc2020.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;

public class TenBallMode extends AutoModeBase {

    //TODO: implement ten ball mode
    private DriveTrajectory mDriveTrajectory;
    @Inject
    private static TrajectorySet trajectorySet;

    public TenBallMode() {
        var trajectory = trajectorySet.TEN_BALL_AUTO;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Ten Ball Auto Trench Mode (Not yet implemented)");
    }
}
