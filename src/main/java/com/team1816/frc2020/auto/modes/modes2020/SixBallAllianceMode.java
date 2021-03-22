package com.team1816.frc2020.auto.modes.modes2020;

import com.team1816.frc2020.auto.actions.actions2020.*;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitUntilInsideRegion;
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
        runAction(
            new SeriesAction(
                new PrepareToShootAction(Turret.CARDINAL_SOUTH),
                new ShootAction(true),
                new ParallelAction(
                    new CollectAction(true),
                    mDriveTrajectory,
                    new TurretAction(15.2),
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                            new Translation2d(78, 68),
                            new Translation2d(180, 88)
                        ),
                        new RampUpShooterAction()
                    )
                ),
                new CollectAction(false),
                new PrepareToShootAction(15.2),
                new ShootAction(true)
            )
        );
    }
}
