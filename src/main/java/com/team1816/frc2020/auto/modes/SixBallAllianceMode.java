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
                new ShootAction(Turret.CARDINAL_SOUTH),
                new ParallelAction(
                    mDriveTrajectory,
                    new SeriesAction(
                       // new WaitUntilInsideRegion(new Translation2d(114, 40), new Translation2d(210, 90)),
                        new CollectAction(true)
                    )
                ),
                new CollectAction(false),
                new ShootAction(Turret.CARDINAL_SOUTH)

            )
        );
    }
}
