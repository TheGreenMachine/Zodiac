package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import java.util.List;

public class BlueRedPathB implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(40, 125, Rotation2d.fromDegrees(0)),
            new Pose2d(150, 58, Rotation2d.fromDegrees(-10)),
            new Pose2d(220, 120, Rotation2d.fromDegrees(10)),
            new Pose2d(335, 55, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
