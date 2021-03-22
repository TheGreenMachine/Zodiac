package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import java.util.List;

public class RedPathA implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            // Loop
            //            new Pose2d(40, 90, Rotation2d.fromDegrees(0)),
            //            new Pose2d(170, 70, Rotation2d.fromDegrees(0)),
            //            new Pose2d(250, 110, Rotation2d.fromDegrees(80)),
            //            new Pose2d(160, 140, Rotation2d.fromDegrees(210)),
            //            new Pose2d(200, 70, Rotation2d.fromDegrees(0)),
            //            new Pose2d(332, 70, Rotation2d.fromDegrees(0))
            // No Loop
            new Pose2d(40, 90, Rotation2d.fromDegrees(0)),
            new Pose2d(145, 65, Rotation2d.fromDegrees(0)),
            new Pose2d(180, 145, Rotation2d.fromDegrees(90))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
