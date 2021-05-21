package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class BlueRedPathB implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(40, 125, 0),
            new Pose2d(97, 125, 0),
            new Pose2d(149, 50, -10),
            new Pose2d(182, 50, 10),
            new Pose2d(197, 128, 0),
            new Pose2d(264, 128, 0),
            new Pose2d(317, 55, 0),
            new Pose2d(339, 55, 0)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
