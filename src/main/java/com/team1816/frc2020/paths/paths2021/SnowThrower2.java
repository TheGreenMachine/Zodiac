package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import java.util.List;

public class SnowThrower2 implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(210, 149, -135),
            new Pose2d(155, 88, -106),
            new Pose2d(160, 31, -90)
        );
    }


    @Override
    public boolean isReversed() {
        return false;
    }
}
