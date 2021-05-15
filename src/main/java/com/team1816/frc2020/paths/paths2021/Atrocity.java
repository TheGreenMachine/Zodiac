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
            new Pose2d(46, 88, 0),
            new Pose2d(79, 103, 45),
            new Pose2d(91, 155, 0),
            new Pose2d(93, 113, -60),
            new Pose2d(116, 77, -60),
            new Pose2d(124, 45, -60),
            new Pose2d(165, 29, 45),
            new Pose2d(179, 79, 90),
            new Pose2d(175, 118, 90),
            new Pose2d(174, 148, 0),
            new Pose2d(186, 120, -90),
            new Pose2d(188, 81, -90),
            new Pose2d(191, 32, -45),
            new Pose2d(254, 32, 45),
            new Pose2d(264, 65, 90),
            new Pose2d(267, 125, 90),
            new Pose2d(271, 158, 0),
            new Pose2d(274, 129, -90),
            new Pose2d(286, 92, -45),
            new Pose2d(325, 89, 0)
        );
    }


    @Override
    public boolean isReversed() {
        return false;
    }
}
