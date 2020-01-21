package com.team1816.frc2020.paths;

import com.team1816.lib.paths.PathBuilder;
import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Hab1ToCargoShipFrontPath implements PathContainer {
    boolean mLeft;

    public Hab1ToCargoShipFrontPath(boolean left) {
        mLeft = left;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return null;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(0, 0, 0, 0));
        sWaypoints.add(new PathBuilder.Waypoint(65, (mLeft ? 1.0 : -1.0) * -20,
                15.0, 100.0));
        sWaypoints.add(new PathBuilder.Waypoint(142.5, (mLeft ? 1.0 : -1.0) * -45, 0, 100.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
