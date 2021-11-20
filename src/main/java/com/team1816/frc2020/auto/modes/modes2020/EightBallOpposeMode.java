package com.team1816.frc2020.auto.modes.modes2020;

import com.team1816.frc2020.auto.actions.actions2020.CollectAction;
import com.team1816.frc2020.auto.actions.actions2020.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.actions2020.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitUntilInsideRegion;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Translation2d;

public class EightBallOpposeMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectoryA;
    private DriveTrajectory mDriveTrajectoryB;
    private Turret turret;

    public EightBallOpposeMode(Turret turret) {
        var trajectoryA = TrajectorySet.getInstance().EIGHT_BALL_AUTO_OPPOSEA;
        var trajectoryB = TrajectorySet.getInstance().EIGHT_BALL_AUTO_OPPOSEB;
        mDriveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        mDriveTrajectoryB = new DriveTrajectory(trajectoryB, true);
        this.turret = turret;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 8 Ball Opposition Side Auto Trench Mode");
        runAction(
            new SeriesAction(
                new ParallelAction(
                    mDriveTrajectoryA,
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                            new Translation2d(70, 0),
                            new Translation2d(125, 0)
                        ),
                        new CollectAction(true),
                        new WaitUntilInsideRegion(
                            new Translation2d(50, 0),
                            new Translation2d(70, 0)
                        ),
                        new CollectAction(false)
                    )
                ),
                new PrepareToShootAction(Turret.MAX_ANGLE, turret),
                new ShootAction(true, turret),
                new ParallelAction(mDriveTrajectoryB, new CollectAction(true)),
                new CollectAction(false),
                new PrepareToShootAction(Turret.MAX_ANGLE, turret),
                new ShootAction(true, turret)
            )
        );
    }
}
