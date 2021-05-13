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
            new Pose2d(150, 30, 90),
            new Pose2d(123, 82, 180),
            new Pose2d(87, 62, -90)
        );
    }


    @Override
    public boolean isReversed() {
        return false;
    }
}
