package com.team1816.frc2020.auto.modes.modes2021;

import com.team1816.frc2020.auto.actions.actions2020.CollectAction;
import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PathAMode extends AutoModeBase {

    private double distance;

    // Blue
    private final DriveTrajectory trajectory;

    // Red -
    private final DriveTrajectory trajectory1;


    public PathAMode() {

        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        networkTable.addEntryListener(
            "distance",
            (table, key, entry, value, flags) -> {
                // Use most recently available distance if distance not found
                this.distance = value.getDouble();
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
        );

        // Blue
        trajectory = new DriveTrajectory(TrajectorySet.getInstance().BLUE_PATHA, true);

        // Red
        trajectory1 = new DriveTrajectory(TrajectorySet.getInstance().RED_PATHA, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Run Path A, distance found was: " + distance);

        if (distance > 0) {
            if (distance > 80) { // RED
                runAction(
                    new SeriesAction(
                        new CollectAction(true),
                        new WaitAction(0.1),
                        trajectory1,
                        new CollectAction(false)
                    )
                );
            } else if (distance < 80){ // BLUE
                runAction(
                    new SeriesAction(
                        new CollectAction(true),
                        new WaitAction(0.1),
                        new ParallelAction(trajectory),
                        new CollectAction(false)
                    )
                );
            }
        }
    }
}
