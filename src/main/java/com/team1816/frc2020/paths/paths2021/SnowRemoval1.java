package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class SnowRemoval1 implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(20, 140, 0),
            new Pose2d(133, 91, 0),
            new Pose2d(211, 151, 45)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
