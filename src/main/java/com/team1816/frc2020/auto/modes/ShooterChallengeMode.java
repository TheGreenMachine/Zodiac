package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.AdjustHoodAction;
import com.team1816.frc2020.auto.actions.CollectAction;
import com.team1816.frc2020.auto.actions.PrepareToShootAction;
import com.team1816.frc2020.auto.actions.ShootAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.trajectory.Trajectory;

public class ShooterChallengeMode extends AutoModeBase {

    private DriveTrajectory mDriveFoward150;
    private DriveTrajectory mDriveFoward210;
    private DriveTrajectory mDriveFoward245A;
    private DriveTrajectory mDriveFoward245B;
    private DriveTrajectory mDriveBackward155;
    private DriveTrajectory mDriveBackward90;
    private DriveTrajectory mDriveBackward150;
    private DriveTrajectory mDriveBackward210;
    private DriveTrajectory mDriveBackward245;

    public ShooterChallengeMode() {
        var trajectoryA = TrajectorySet.getInstance().DriveStraightCustom(149);
        var trajectoryB = TrajectorySet.getInstance().DriveStraightCustom(209);
        var trajectoryC = TrajectorySet.getInstance().DriveStraightCustom(249);
        var trajectoryH = TrajectorySet.getInstance().DriveStraightCustom(249);
        var trajectoryD = TrajectorySet.getInstance().DriveStraightCustom(-155);
        var trajectoryE = TrajectorySet.getInstance().DriveStraightCustom(-89);
        var trajectoryF = TrajectorySet.getInstance().DriveStraightCustom(-150);
        var trajectoryG = TrajectorySet.getInstance().DriveStraightCustom(-210);
        var trajectoryI = TrajectorySet.getInstance().DriveStraightCustom(-245);
        mDriveFoward150 = new DriveTrajectory(trajectoryA, true);
        mDriveFoward210 = new DriveTrajectory(trajectoryB, true);
        mDriveFoward245A = new DriveTrajectory(trajectoryC, true);
        mDriveFoward245B  = new DriveTrajectory(trajectoryH, true);
        mDriveBackward155 = new DriveTrajectory(trajectoryD, true);
        mDriveBackward90 = new DriveTrajectory(trajectoryE, true);
        mDriveBackward150 = new DriveTrajectory(trajectoryF, true);
        mDriveBackward210 = new DriveTrajectory(trajectoryG, true);
        mDriveBackward245 = new DriveTrajectory(trajectoryI, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Shooter Challenge Mode");
        runAction(
            new SeriesAction(
                new AdjustHoodAction(true),
                new ParallelAction(
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,5),
                    mDriveBackward155
                ),
                new ShootAction(4, true, 5),
                mDriveBackward90,
                new WaitAction(3),

                new ParallelAction(
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,3),
                    mDriveFoward150
                ),
                new ShootAction(4, true, 3),
                mDriveBackward150,
                new WaitAction(3),

                new ParallelAction(
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,2),
                    mDriveFoward210
                ),
                new ShootAction(4, true, 2),
                mDriveBackward210,
                new WaitAction(3),

                new ParallelAction(
                    mDriveFoward245A,
                    new AdjustHoodAction(false),
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,1)
                ),
                new ShootAction(4, true, 1),
                mDriveBackward245,
                new WaitAction(3),

                new ParallelAction(
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,1),
                    mDriveFoward245B
                ),
                new ShootAction(4, true, 1)



            )
        );
    }
}
