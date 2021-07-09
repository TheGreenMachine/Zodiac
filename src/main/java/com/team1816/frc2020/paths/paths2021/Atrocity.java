package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class Atrocity implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(39, 98, 0),
            new Pose2d(172, 103, 0),
            new Pose2d(149, 6, 180),
            new Pose2d(132, 96, 10),
            new Pose2d(278, 82, 30),
            new Pose2d(242, 169, 176),
            new Pose2d(190, 136, -117),
            new Pose2d(203, 89, -60),
            new Pose2d(271, 27, -25),
            new Pose2d(338, 93, 132),
            new Pose2d(242, 111, 180),
            new Pose2d(146, 127, 180),
            new Pose2d(37, 136, 180)
        );
    }


    @Override
    public boolean isReversed() {
        return false;
    }
}
