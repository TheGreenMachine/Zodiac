package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class BouncePath3 implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(187, 148, -91),
            new Pose2d(189, 106, -90),
            new Pose2d(224, 33, 0),
            new Pose2d(276, 112, 90),
            new Pose2d(277, 146, 90)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
