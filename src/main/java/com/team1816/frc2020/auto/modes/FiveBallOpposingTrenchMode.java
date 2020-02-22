package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Translation2d;

public class FiveBallOpposingTrenchMode extends AutoModeBase {
    private DriveTrajectory mDriveTrajectory;

    public FiveBallOpposingTrenchMode() {
        var trajectory = TrajectorySet.getInstance().FIVE_BALL_AUTO_OPPOSE;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Five Ball Opposition Auto Trench Mode");
        runAction(new WaitAction(.5));
        // runAction(mDriveTrajectory);
        runAction(
            new SeriesAction(
                new ParallelAction(
                    mDriveTrajectory,
                    new SeriesAction(
                        new WaitUntilInsideRegion(new Translation2d(50, 20), new Translation2d(100, -20)),
                        new CollectAction(true),
                        new WaitUntilInsideRegion(new Translation2d(100,35),new Translation2d(70,75)),
                        new CollectAction(false)
                    )
                ),
                new ShootAction(Turret.CARDINAL_WEST)
            )
        );
    }
}
