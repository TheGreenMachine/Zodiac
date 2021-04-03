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
            new Pose2d(179, 107, 0),
            new Pose2d(273, 52, -60),
            new Pose2d(338, 63, 90),
            new Pose2d(275, 74, -120),
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
