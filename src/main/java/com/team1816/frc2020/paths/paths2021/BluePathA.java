package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class BluePathA implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(40, 30, 0),
            new Pose2d(197, 91, 80),
            new Pose2d(330, 55, -30)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
