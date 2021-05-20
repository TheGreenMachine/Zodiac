package com.team1816.frc2020.auto.modes.modes2021;

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

public class SnowThrowerMode extends AutoModeBase {
    private DriveTrajectory trajectory1;
    private DriveTrajectory trajectory2;
    private DriveTrajectory trajectory3;

    public SnowThrowerMode() {
        trajectory1 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_THROWER_1, true);
        trajectory2 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_THROWER_2, false);
        trajectory3 = new DriveTrajectory(TrajectorySet.getInstance().SNOW_THROWER_3, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new RampUpShooterAction(),
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                            new Translation2d(140, 80),
                            new Translation2d(240, 180)
                        ),
                        new CollectAction(true)
                    ),
                    new SeriesAction(
                        new TurretAction(Turret.MAX_ANGLE),
                        trajectory1,
                        new CollectAction(false),
                        new ShootAction(2, false)
                    )
                ),
                new ParallelAction(
                    new TurretModeAction(Turret.ControlMode.FIELD_FOLLOWING),
                    new RampUpShooterAction(),
                    // new TurretAction(220),
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                           new Translation2d(120, 0),
                           new Translation2d(210, 90)
                        ),
                        new CollectAction(true)
                    ),
                    new SeriesAction(
                        trajectory2,
                        new ShootAction(2, false, 9000)
                    )
                ),
                new ParallelAction(
                    new RampUpShooterAction(),
                    new CollectAction(true),
                    // new TurretAction(220),
                    new SeriesAction(
//                        new WaitUntilInsideRegion(
//                            new Translation2d(78, 68),
//                            new Translation2d(180, 88)
//                        ),
                        trajectory3,
                        new ShootAction(2, false)
                    )
                )
            )
        );
    }
}
