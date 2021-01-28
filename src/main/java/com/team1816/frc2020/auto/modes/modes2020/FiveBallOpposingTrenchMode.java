package com.team1816.frc2020.auto.modes.modes2020;

import com.team1816.frc2020.auto.actions.actions2020.CollectAction;
import com.team1816.frc2020.auto.actions.actions2020.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.actions2020.ShootAction;
import com.team1816.frc2020.paths.paths2020.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class FiveBallOpposingTrenchMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectoryA;
    private DriveTrajectory mDriveTrajectoryB;

    public FiveBallOpposingTrenchMode() {
        var trajectoryA = TrajectorySet.getInstance().FIVE_BALL_AUTO_OPPOSEA;
        var trajectoryB = TrajectorySet.getInstance().FIVE_BALL_AUTO_OPPOSEB;
        mDriveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        mDriveTrajectoryB = new DriveTrajectory(trajectoryB, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Five Ball Opposition Auto Trench Mode");
        runAction(
            new SeriesAction(
                new ParallelAction(mDriveTrajectoryA, new CollectAction(true)),
                new CollectAction(false),
                new ParallelAction(
                    mDriveTrajectoryB,
                    new PrepareToShootAction(Turret.CARDINAL_SOUTH)
                ),
                new ShootAction(true)
            )
        );
    }
}
