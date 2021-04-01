package com.team1816.frc2020.auto.modes;

import com.team1816.frc2020.auto.actions.*;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.trajectory.Trajectory;

public class ShooterChallengeMode2 extends AutoModeBase {

    private DriveTrajectory mDriveFoward100A;
    private DriveTrajectory mDriveFoward100B;
    private DriveTrajectory mDriveFoward100C;
    private DriveTrajectory mDriveFoward100D;
    private DriveTrajectory mDriveFoward100E;
    private DriveTrajectory mDriveFoward100F;
    private DriveTrajectory mDriveFoward100G;
    private DriveTrajectory mDriveFoward100H;
    private DriveTrajectory mDriveBackward100A;
    private DriveTrajectory mDriveBackward100B;
    private DriveTrajectory mDriveBackward100C;
    private DriveTrajectory mDriveBackward100D;
    private DriveTrajectory mDriveBackward100E;
    private DriveTrajectory mDriveBackward100F;
    private DriveTrajectory mDriveBackward100G;
    private DriveTrajectory mDriveBackward100H;




    public ShooterChallengeMode2(){
        mDriveBackward100A = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveBackward100B = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveBackward100C = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveBackward100D = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveBackward100E = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveBackward100F = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveBackward100G = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveBackward100H = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(-100), true);
        mDriveFoward100A = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(100),true);
        mDriveFoward100B = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(100), true);
        mDriveFoward100C = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(101),true);
        mDriveFoward100D = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(100), true);
        mDriveFoward100E = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(101), true);
        mDriveFoward100F = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(100), true);
        mDriveFoward100G = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(100), true);
        mDriveFoward100H = new DriveTrajectory(TrajectorySet.getInstance().DriveStraightCustom(100), true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 2nd Shooter Challenge Mode");
        runAction(
            new SeriesAction(

                new AdjustHoodAction(true),
                new PrepareToShootAction(Turret.CARDINAL_NORTH,4),
                new ShootAction(1.2, true, 4),
                mDriveBackward100A,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100A,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                    ),
                new ShootAction(1.2, true, 4),
                mDriveBackward100B,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100B,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                ),
                new ShootAction(1.2, true, 4),
                mDriveBackward100C,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100C,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                ),
                new ShootAction(1.2, true, 4),
                mDriveBackward100D,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100D,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                ),
                new ShootAction(1.2, true, 4),
                mDriveBackward100E,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100E,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                ),
                new ShootAction(1.2, true, 4),
                mDriveBackward100F,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100F,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                ),
                new ShootAction(1.2, true, 4),
                mDriveBackward100G,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100G,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                ),
                new ShootAction(1.2, true, 4),
                mDriveBackward100H,
                new WaitAction(0.6),
                new ParallelAction(
                    mDriveFoward100H,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH,4, true)
                ),
                new ShootAction(1.2, true, 4)




            )
        );
    }
}
