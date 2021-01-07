package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.auto.actions.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class EightBallAllianceMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectoryA;
    private DriveTrajectory mDriveTrajectoryB;
    private DriveTrajectory mDriveTrajectoryC;
    private DriveTrajectory mDriveTrajectoryD;

    public EightBallAllianceMode() {
        var trajectoryA = TrajectorySet.getInstance().DRIVE_STRAIGHT_TRENCH;
        var trajectoryB = TrajectorySet.getInstance().EIGHT_BALL_AUTO_ALLIANCEA;
        var trajectoryC = TrajectorySet.getInstance().EIGHT_BALL_AUTO_ALLIANCEB;
        var trajectoryD = TrajectorySet.getInstance().EIGHT_BALL_AUTO_ALLIANCEC;
        mDriveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        mDriveTrajectoryB = new DriveTrajectory(trajectoryB, true);
        mDriveTrajectoryC = new DriveTrajectory(trajectoryC, true);
        mDriveTrajectoryD = new DriveTrajectory(trajectoryD, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 8 Ball Alliance Side Auto Trench Mode");
        runAction(
            new SeriesAction(
                new PrepareToShootAction(Turret.CARDINAL_NORTH),
                new ShootAction(2.5, false),
                new ParallelAction(
                    new CollectAction(true),
                    new SeriesAction(
                        mDriveTrajectoryA,
                        mDriveTrajectoryB,
                        mDriveTrajectoryC,
                        new ParallelAction(
                            mDriveTrajectoryD,
                            new CollectAction(false),
                            new PrepareToShootAction(Turret.CARDINAL_SOUTH)
                        )
                    )
                ),
                new ShootAction(true)
            )
        );
    }
}
