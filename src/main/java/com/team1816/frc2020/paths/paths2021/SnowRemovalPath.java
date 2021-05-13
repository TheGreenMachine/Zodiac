package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.List;

public class SnowRemovalPath implements PathContainer {
    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(28, 151, 0),
            new Pose2d(80, 124, -18),
            new Pose2d(153, 97, -11),
            new Pose2d(184, 130, 59),
            new Pose2d(212, 159, 3),
            new Pose2d(175, 107, -100),
            new Pose2d(168, 73, -100),
            new Pose2d(150, 28, -158),
            new Pose2d(114, 27, 165),
            new Pose2d(97, 60, 5),
            new Pose2d(184, 53, 0),
            new Pose2d(241, 66, 40),
            new Pose2d(268, 118, 70),
            new Pose2d(299, 149, 0),
            new Pose2d(325, 124, -90),
            new Pose2d(281, 77, -110),
            new Pose2d(303, 41, 0),
            new Pose2d(325, 60, 90),
            new Pose2d(302, 88, 180),
            new Pose2d(209, 61, 180),
            new Pose2d(70, 83, -170),
            new Pose2d(29, 60, -125),
            new Pose2d(27, 18, -90)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
