package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class SnowRemoval3 implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(90, 54, 0),
            new Pose2d(218, 56, 5),
            new Pose2d(300, 155, 0),
            new Pose2d(325, 119, -90),
            new Pose2d(249, 68, -51),
            new Pose2d(299, 39, 0),
            new Pose2d(330, 60, 90),
            new Pose2d(300, 96, 180),
            new Pose2d(211, 61, 180),
            new Pose2d(80, 101, 180),
            new Pose2d(20, 42, -100)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
