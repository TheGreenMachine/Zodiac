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
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class ShooterChallengeMode extends AutoModeBase {

    private DriveTrajectory mDriveFoward235;
    private DriveTrajectory mDriveFoward175;
    private DriveTrajectory mDriveFoward115;
    private DriveTrajectory mDriveBackward270;
    private DriveTrajectory mDriveBackward235;
    private DriveTrajectory mDriveBackward175;
    private DriveTrajectory mDriveBackward115;

    public ShooterChallengeMode() {
        var trajectoryA = TrajectorySet.getInstance().DriveStraightCustom(235);
        var trajectoryB = TrajectorySet.getInstance().DriveStraightCustom(175);
        var trajectoryC = TrajectorySet.getInstance().DriveStraightCustom(115);
        var trajectoryD = TrajectorySet.getInstance().DriveStraightCustom(-270);
        var trajectoryE = TrajectorySet.getInstance().DriveStraightCustom(-235);
        var trajectoryF = TrajectorySet.getInstance().DriveStraightCustom(-175);
        var trajectoryG = TrajectorySet.getInstance().DriveStraightCustom(-115);
        mDriveFoward235 = new DriveTrajectory(trajectoryA, true);
        mDriveFoward175 = new DriveTrajectory(trajectoryB, true);
        mDriveFoward115 = new DriveTrajectory(trajectoryC, true);
        mDriveBackward270 = new DriveTrajectory(trajectoryD, true);
        mDriveBackward235 = new DriveTrajectory(trajectoryE, true);
        mDriveBackward175 = new DriveTrajectory(trajectoryF, true);
        mDriveBackward115 = new DriveTrajectory(trajectoryG, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Shooter Challenge Mode");
        runAction(
            new SeriesAction(
                new PrepareToShootAction(Turret.CARDINAL_NORTH),
                new ShootAction(2.5, false),
                mDriveBackward270,
                new WaitAction(3),
                    new ParallelAction(
                      mDriveFoward235,
                      new PrepareToShootAction(Turret.CARDINAL_NORTH)
                    ),
                new ShootAction(true),
                mDriveBackward235,
                new WaitAction(3),
                new ParallelAction(
                    mDriveFoward175,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH)
                ),
                new ShootAction(true),
                mDriveBackward175,
                new WaitAction(3),
                new ParallelAction(
                    mDriveFoward115,
                    new PrepareToShootAction(Turret.CARDINAL_NORTH)
                ),
                new ShootAction(true),
                mDriveBackward115

            )
        );
    }
}
