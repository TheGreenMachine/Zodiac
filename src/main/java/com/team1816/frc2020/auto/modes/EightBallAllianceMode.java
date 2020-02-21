package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Translation2d;

public class EightBallAllianceMode extends AutoModeBase {
    private DriveTrajectory mDriveTrajectory;

    public EightBallAllianceMode(boolean turnRight) {
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
                new ShootAction(Turret.CARDINAL_NORTH),
                new ParallelAction(
                    mDriveTrajectory,
                    new CollectAction(true)
                ),
                new CollectAction(false),
                new ShootAction(Turret.CARDINAL_EAST)

            )
        );
    }
}
