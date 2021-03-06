package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class UTurnPath implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(90, 41, 0),
            new Pose2d(240, 41, 0),
            new Pose2d(309, 96, 90),
            new Pose2d(244, 151, 180),
            new Pose2d(46, 150, 180)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
