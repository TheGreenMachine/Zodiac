package com.team1816.frc2020.paths;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class LivingRoomPath implements PathContainer {

    @Override
    public Path buildPath() {
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(79.5, 11.0, Rotation2d.fromDegrees(45)));
        waypoints.add(new Pose2d(172, 30, Rotation2d.fromDegrees(0)));
        return waypoints;
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
