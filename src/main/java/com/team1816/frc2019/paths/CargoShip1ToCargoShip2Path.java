package com.team1816.frc2019.paths;

import com.team1816.lib.paths.PathBuilder;
import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class CargoShip1ToCargoShip2Path implements PathContainer {
    public static final String kTurnTurretMarker = "READY_TO_TURN";

    boolean mLeft;

    public CargoShip1ToCargoShip2Path(boolean left) {
        mLeft = left;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return null;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(195, (mLeft ? 1.0 : -1.0) * 0, 0, 100));
        sWaypoints.add(new PathBuilder.Waypoint(220, (mLeft ? 1.0 : -1.0) * 0, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
