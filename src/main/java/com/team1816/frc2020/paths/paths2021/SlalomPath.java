package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class SlalomPath implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(20, 30, 0),
            new Pose2d(95, 73, 55),
            new Pose2d(171, 123, 0),
            new Pose2d(197, 123, 0),
            new Pose2d(280, 41, -60),
            new Pose2d(339, 64, 90),
            new Pose2d(278, 79, -120),
            new Pose2d(180, 24, 180),
            new Pose2d(116, 33, 150),
            new Pose2d(73, 85, 125)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
