package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class SnowThrower1 implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(38, 138, 0),
            new Pose2d(127, 98, -30),
            new Pose2d(169, 99, 60),
            new Pose2d(210, 149, 15)
        );
    }


    @Override
    public boolean isReversed() {
        return false;
    }
}
