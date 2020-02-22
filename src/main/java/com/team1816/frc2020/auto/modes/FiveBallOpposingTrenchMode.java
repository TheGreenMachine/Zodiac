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
import com.team254.lib.trajectory.Trajectory;

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
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    mDriveTrajectoryA,
                    new CollectAction(true)
                ),
                new CollectAction(false),
                new ParallelAction(
                    mDriveTrajectoryB,
                    new RampUpShooterAction(Turret.CARDINAL_SOUTH)
                ),
                new ShootAction(Turret.CARDINAL_SOUTH)
            )
        );
    }
}
