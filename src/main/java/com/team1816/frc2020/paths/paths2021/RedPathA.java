package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class RedPathA implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(40, 90, 0),
            new Pose2d(145, 65, 0),
            new Pose2d(165, 165, 0),
            new Pose2d(347, 165, 0)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
