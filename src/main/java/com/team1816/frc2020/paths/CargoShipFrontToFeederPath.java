package com.team1816.frc2020.paths;

import com.team1816.lib.paths.PathBuilder;
import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class CargoShipFrontToFeederPath implements PathContainer {
    public static final String kLookForTargetMarker = "LOOK_FOR_TARGET";

    boolean mLeft;

    public CargoShipFrontToFeederPath(boolean left) {
        mLeft = left;
    }


    @Override
    public List<Pose2d> buildWaypoints() {
        return null;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(142.5, (mLeft ? 1.0 : -1.0) * -45, 0, 0));
        sWaypoints.add(new PathBuilder.Waypoint(100, (mLeft ? 1.0 : -1.0) * 20, 40, 100));
        sWaypoints.add(new PathBuilder.Waypoint(55, (mLeft ? 1.0 : -1.0) * 80, 0, 100, kLookForTargetMarker));
        sWaypoints.add(new PathBuilder.Waypoint(-5, (mLeft ? 1.0 : -1.0) * 80, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
