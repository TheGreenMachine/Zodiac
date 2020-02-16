package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Translation2d;

public class AutoTrenchMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectory;

    public AutoTrenchMode(boolean turnRight) {
        var trajectory = TrajectorySet.getInstance().AUTO_TRENCH;

        if (turnRight) {
            trajectory = TrajectorySet.getInstance().AUTO_TRENCH_TURN_RIGHT;
        }

        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Auto Trench Mode");
        runAction(new WaitAction(.5));
        // runAction(mDriveTrajectory);
        runAction(
            new SeriesAction(
               // new ShootAction(),
                new ParallelAction(
                    mDriveTrajectory,
                    new SeriesAction(
                        new WaitUntilInsideRegion(new Translation2d(114, 40), new Translation2d(210, 90)),
                        new CollectAction(true)
                    )
                ),
                new CollectAction(false)
            )
        );
    }
}
