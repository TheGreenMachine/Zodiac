package com.team1816.frc2020.paths;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.Arrays;
import java.util.List;

public class FeederToTrenchPath implements PathContainer {
    @Override
    public Path buildPath() {
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return Arrays.asList(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(190, -70, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
