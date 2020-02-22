package com.team1816.frc2020.paths;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

public class DriveStraight implements PathContainer {
    private int driveDistance;

    public DriveStraight(int driveDistance) {
        this.driveDistance = driveDistance;
    }

    public DriveStraight() {
        this(60);
    }

    @Override
    public Path buildPath() {
      /*
      ArrayList<PathBuilder.Waypoint> waypoints = new ArrayList<>();
      waypoints.add(new PathBuilder.Waypoint(0, 0, 0, 0);
      waypoints.add(new PathBuilder.Waypoint(driveDistance, 0,0, 0));
      return  PathBuilder.buildPathFromWaypoints(waypoints);
       */
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(driveDistance, 0.0, Rotation2d.fromDegrees(0)));
        return waypoints;
    }

    @Override
    public boolean isReversed() {
        return driveDistance < 0;
    }
}
