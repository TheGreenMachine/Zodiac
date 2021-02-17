package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class BarrelPath implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(39, 93, 0),
            new Pose2d(180, 80, -45),
            new Pose2d(120, 30, 150),
            new Pose2d(160, 90, 10),
            new Pose2d(270, 100, 30),
            new Pose2d(250, 150, 180),
            new Pose2d(220, 96, -60),
            new Pose2d(300, 31, 0),
            new Pose2d(300, 93, 180),
            new Pose2d(39, 93, 180)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
