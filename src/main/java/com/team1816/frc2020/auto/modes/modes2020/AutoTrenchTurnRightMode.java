package com.team1816.frc2020.auto.modes.modes2020;

import com.team1816.frc2020.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;

public class AutoTrenchTurnRightMode extends AutoModeBase {

    private DriveTrajectory trajectory;

    public AutoTrenchTurnRightMode() {
        this.trajectory = new DriveTrajectory(TrajectorySet.getInstance().AUTO_TRENCH_TURN_RIGHT, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(trajectory);
    }
}
