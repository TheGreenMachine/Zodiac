package com.team1816.frc2020.auto.modes.modes2020;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class TuneDrivetrainMode extends AutoModeBase {

    private DriveTrajectory trajectory;

    public TuneDrivetrainMode() {
        trajectory =
            new DriveTrajectory(TrajectorySet.getInstance().TUNE_DRIVETRAIN, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Tune Drivetrain path");
        runAction(new WaitAction(1));
        runAction(trajectory);
    }
}
