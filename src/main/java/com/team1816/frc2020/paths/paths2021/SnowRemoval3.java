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
            new Pose2d(300, 145, 0),
            new Pose2d(322, 121, -90),
            new Pose2d(274, 59, -90),
            new Pose2d(299, 39, 0),
            new Pose2d(325, 60, 90),
            new Pose2d(297, 83, 180),
            new Pose2d(211, 61, 180),
            new Pose2d(84, 83, 180),
            new Pose2d(30, 45, -100)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
