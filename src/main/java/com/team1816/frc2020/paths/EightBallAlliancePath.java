package com.team1816.frc2020.paths;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.List;

public class EightBallAlliancePath implements PathContainer {
    @Override
    public Path buildPath() {
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d(114,65, Rotation2d.fromDegrees(0)),
            new Pose2d(200,65, Rotation2d.fromDegrees(0)),
            new Pose2d(150,0, Rotation2d.fromDegrees(80))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
