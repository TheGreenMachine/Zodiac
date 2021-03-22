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
            new Pose2d(80, 36, 55),
            new Pose2d(180, 116, 0),
            new Pose2d(270, 60, -60),
            new Pose2d(330, 66, 90),
            new Pose2d(270, 60, 240),
            new Pose2d(180, 24, 180),
            new Pose2d(110, 40, 150),
            new Pose2d(50, 96, 160)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
