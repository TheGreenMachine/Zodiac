package com.team1816.frc2019.auto.modes;

import com.team1816.frc2019.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class LivingRoomMode  extends AutoModeBase {

    private DriveTrajectory mDriveTrajectory;

    public LivingRoomMode() {
        var trajectory = TrajectorySet.getInstance().LIVING_ROOM;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");;
        runAction(new WaitAction(.5));
        runAction(mDriveTrajectory);
    }
}
