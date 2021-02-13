package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import java.util.List;

public class DimeTurn implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(180, 145, Rotation2d.fromDegrees(0)),
            new Pose2d(330, 145, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
