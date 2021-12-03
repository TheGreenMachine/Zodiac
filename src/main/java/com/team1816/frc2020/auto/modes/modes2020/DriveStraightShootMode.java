package com.team1816.frc2020.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.frc2020.auto.actions.actions2020.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.actions2020.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class DriveStraightShootMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectory;
    @Inject
    private static TrajectorySet trajectorySet;

    public DriveStraightShootMode() {
        var trajectory = trajectorySet.DRIVE_STRAIGHT;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                mDriveTrajectory,
                new PrepareToShootAction(Turret.CARDINAL_SOUTH),
                new ShootAction(false)
            )
        );
    }
}
