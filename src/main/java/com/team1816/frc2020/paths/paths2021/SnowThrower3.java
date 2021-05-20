package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import java.util.List;

public class SnowThrower3 implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(160, 32, 85),
            new Pose2d(119, 92, 180),
            new Pose2d(92, 56, -94)
        );
    }


    @Override
    public boolean isReversed() {
        return false;
    }
}
