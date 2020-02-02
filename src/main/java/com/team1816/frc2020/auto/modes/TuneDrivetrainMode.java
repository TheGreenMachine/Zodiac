package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.paths.DriveStraight;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DrivePathAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class TuneDrivetrainMode extends AutoModeBase {

    private DrivePathAction mPath;

    public TuneDrivetrainMode() {
        mPath = new DrivePathAction(new DriveStraight(), true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Tune Drivetrain path");
        runAction(new WaitAction(1));
        runAction(mPath);
    }
}
