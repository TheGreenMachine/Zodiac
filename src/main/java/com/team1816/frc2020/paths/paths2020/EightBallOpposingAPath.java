package com.team1816.frc2020.paths.paths2020;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import java.util.List;

public class EightBallOpposingAPath implements PathContainer {

    @Override
    public Path buildPath() {
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of( // TODO: Needs to be tested
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(125, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(30, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 40, Rotation2d.fromDegrees(90))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
