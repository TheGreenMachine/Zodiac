package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class BouncePath2 implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(91, 140, 90),
            new Pose2d(115, 66, 110),
            new Pose2d(149, 37, -180),
            new Pose2d(189, 97, -90),
            new Pose2d(187, 148, -90)
        );
    }

    @Override
    public boolean isReversed() { // to make the robot go backwards this needs to be true
        return true;
    }
}
