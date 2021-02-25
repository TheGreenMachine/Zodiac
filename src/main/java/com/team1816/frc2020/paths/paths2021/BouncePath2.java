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
            new Pose2d(91, 140, 90),
            new Pose2d(110, 77, 100),
            new Pose2d(154, 27, -160),
            new Pose2d(179, 84, -90),
            new Pose2d(177, 148, -90)
        );
    }

    @Override
    public boolean isReversed() { // to make the robot go backwards this needs to be true
        return true;
    }
}
