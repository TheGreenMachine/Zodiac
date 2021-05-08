package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import java.util.List;

public class BarrelPath implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        /*
        return List.of(
            new Pose2d(39, 93, 0),
            new Pose2d(150, 101, 0),
            new Pose2d(149, 25, 180),
            new Pose2d(143, 99, 10),
            new Pose2d(266, 83, 30),
            new Pose2d(248, 156, 176),
            new Pose2d(208, 90, -60),
            new Pose2d(275, 41, -25),
            new Pose2d(326, 94, 134),
            new Pose2d(208, 95, 180),
            new Pose2d(143, 95, 180),
            new Pose2d(39, 96, 180)
        );
         */
        return List.of(
            new Pose2d(0, 0, -90),
            new Pose2d(50, -50, 0),
            new Pose2d(100, 0, 90),
            new Pose2d(50, 50, 180),
            new Pose2d(0, 0, -90),
            new Pose2d(50, -50, 0),
            new Pose2d(100, 0, 90),
            new Pose2d(50, 50, 180),
            new Pose2d(0, 0, -90),
            new Pose2d(50, -50, 0),
            new Pose2d(100, 0, 90),
            new Pose2d(50, 50, 180),
            new Pose2d(0, 0, -90)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
