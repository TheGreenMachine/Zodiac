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
            new Pose2d(40, 120, Rotation2d.fromDegrees(0)),
            new Pose2d(100, 120, Rotation2d.fromDegrees(-20)),
            new Pose2d(160, 60, Rotation2d.fromDegrees(-20)),
            new Pose2d(220, 120, Rotation2d.fromDegrees(20)),
            new Pose2d(310, 45, Rotation2d.fromDegrees(-65))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
