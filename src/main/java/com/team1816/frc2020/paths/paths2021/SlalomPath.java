package com.team1816.frc2020.paths.paths2021;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import java.util.List;

public class SlalomPath implements PathContainer {

    @Override
    public Path buildPath() {
        return null;
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(21, 21, 0),
            new Pose2d(87, 60, 85),
            new Pose2d(175, 127, 0),
            new Pose2d(267, 63, -90),
            new Pose2d(307, 9, 0),
            new Pose2d(359, 59, 90),
            new Pose2d(307, 114, 180),
            new Pose2d(265, 62, -90),
            new Pose2d(179, 11, 180),
            new Pose2d(74, 61, 90),
            new Pose2d(38, 97, 180)
        );
        /*// ====== FIGURE EIGHT ^^ =====
        List.of(
            new Pose2d(0, 0, -90),
            new Pose2d(60, -44, 0),
            new Pose2d(180, 44, 0),
            new Pose2d(240, 0, -90),
            new Pose2d(180, -44, 180),
            new Pose2d(60, 44, 180),
            new Pose2d(0, 0, -90)
        );*/
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
