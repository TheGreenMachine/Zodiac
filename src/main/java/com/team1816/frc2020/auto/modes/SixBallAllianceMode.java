package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.auto.actions.RampUpShooterAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Translation2d;

public class SixBallAllianceMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectory;

    public SixBallAllianceMode() {
        var trajectory = TrajectorySet.getInstance().SIX_BALL_ALLIANCE;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Six Ball Alliance Auto Trench Mode");
        runAction(new WaitAction(.5));
        // runAction(mDriveTrajectory);
        runAction(
            new SeriesAction(
                new RampUpShooterAction(Turret.CARDINAL_SOUTH),
                new ShootAction(Turret.CARDINAL_SOUTH),
                new ParallelAction(
                    new CollectAction(true),
                    mDriveTrajectory,
                    new RampUpShooterAction(15.2)
                ),
                new CollectAction(false),
                new ShootAction(15.2)
            )
        );
    }
}
