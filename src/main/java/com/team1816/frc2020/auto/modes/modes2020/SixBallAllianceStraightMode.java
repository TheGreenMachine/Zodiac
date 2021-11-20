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

public class SixBallAllianceStraightMode extends AutoModeBase {

    private DriveTrajectory driveTrajectoryA;
    private DriveTrajectory driveTrajectoryB;
    private Turret turret;

    public SixBallAllianceStraightMode(Turret turret) {
        var trajectoryA = TrajectorySet.getInstance().DRIVE_STRAIGHT_TRENCH;
        var trajectoryB = TrajectorySet.getInstance().DRIVE_STRAIGHT_TRENCH_REVERSE;
        this.driveTrajectoryA = new DriveTrajectory(trajectoryA, true);
        this.driveTrajectoryB = new DriveTrajectory(trajectoryB, true);
        this.turret = turret;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new PrepareToShootAction(0, turret),
                new ShootAction(2.5, false, turret),
                new ParallelAction(new CollectAction(true), driveTrajectoryA),
                new CollectAction(false),
                new ParallelAction(new PrepareToShootAction(15.2, turret), driveTrajectoryB),
                new ShootAction(4, true, 9_300, turret)
            )
        );
    }
}
