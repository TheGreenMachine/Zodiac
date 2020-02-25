package com.team1816.frc2020.paths;

import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.List;

public class FiveBallOpposingPathPartB implements PathContainer {
    @Override
    public Path buildPath() {
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(

            new Pose2d(0,0, Rotation2d.fromDegrees(0)),
            new Pose2d(-75,100, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
