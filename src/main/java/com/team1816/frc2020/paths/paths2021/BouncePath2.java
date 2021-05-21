package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class BouncePath2 implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(91, 150, -90),
            new Pose2d(115, 87, -40),
            new Pose2d(145, 22, 0),
            new Pose2d(164, 21, 0),
            new Pose2d(190, 78, 90),
            new Pose2d(190, 149, 90)
        );
    }

    @Override
    public boolean isReversed() { // to make the robot go backwards this needs to be true
        return false;
    }
}
