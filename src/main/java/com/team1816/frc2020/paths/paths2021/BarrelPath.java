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
            new Pose2d(150, 101, 0),
            new Pose2d(150, 22, 180),
            new Pose2d(143, 99, 10),
            new Pose2d(278, 87, 30),
            new Pose2d(255, 158, 176),
            new Pose2d(212, 95, -60),
            new Pose2d(271, 27, -25),
            new Pose2d(326, 94, 132),
            new Pose2d(208, 86, 180),
            new Pose2d(39, 93, 180)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
