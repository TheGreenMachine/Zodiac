package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class EightBallAllianceMode extends AutoModeBase {
    private DriveTrajectory mDriveTrajectoryA;
    private DriveTrajectory mDriveTrajectoryB;

    public EightBallAllianceMode() {
        var trajectoryA = TrajectorySet.getInstance().SIX_BALL_ALLIANCE;
        var trajectoryB = TrajectorySet.getInstance().EIGHT_BALL_AUTO_ALLIANCE;
        mDriveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        mDriveTrajectoryB = new DriveTrajectory(trajectoryB, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 8 Ball Alliance Side Auto Trench Mode");
        runAction(new WaitAction(.5));
        // runAction(mDriveTrajectory);
        runAction(
            new SeriesAction(
                new ShootAction(Turret.CARDINAL_NORTH),
                new ParallelAction(
                    new SeriesAction(
                        mDriveTrajectoryA,
                        mDriveTrajectoryB
                    ),
                    new SeriesAction(
                        new CollectAction(true)
                    )
                ),
                new CollectAction(false),
                new ShootAction(Turret.MAX_ANGLE)
            )
        );
    }
}
