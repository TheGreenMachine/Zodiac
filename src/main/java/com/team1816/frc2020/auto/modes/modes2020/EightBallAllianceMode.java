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
import com.team1816.lib.auto.modes.AutoModeBase;

public class EightBallAllianceMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectoryA;
    private DriveTrajectory mDriveTrajectoryB;
    private DriveTrajectory mDriveTrajectoryC;
    private DriveTrajectory mDriveTrajectoryD;
    private Turret turret;

    public EightBallAllianceMode(Turret turret) {
        var trajectoryA = TrajectorySet.getInstance().DRIVE_STRAIGHT_TRENCH;
        var trajectoryB = TrajectorySet.getInstance().EIGHT_BALL_AUTO_ALLIANCEA;
        var trajectoryC = TrajectorySet.getInstance().EIGHT_BALL_AUTO_ALLIANCEB;
        var trajectoryD = TrajectorySet.getInstance().EIGHT_BALL_AUTO_ALLIANCEC;
        mDriveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        mDriveTrajectoryB = new DriveTrajectory(trajectoryB, true);
        mDriveTrajectoryC = new DriveTrajectory(trajectoryC, true);
        mDriveTrajectoryD = new DriveTrajectory(trajectoryD, true);
        this.turret = turret;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 8 Ball Alliance Side Auto Trench Mode");
        runAction(
            new SeriesAction(
                new PrepareToShootAction(Turret.CARDINAL_NORTH, turret),
                new ShootAction(2.5, false, turret),
                new ParallelAction(
                    new CollectAction(true),
                    new SeriesAction(
                        mDriveTrajectoryA,
                        mDriveTrajectoryB,
                        mDriveTrajectoryC,
                        new ParallelAction(
                            mDriveTrajectoryD,
                            new CollectAction(false),
                            new PrepareToShootAction(Turret.CARDINAL_SOUTH, turret)
                        )
                    )
                ),
                new ShootAction(true, turret)
            )
        );
    }
}
