package com.team1816.frc2019.paths;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class TrajectorySet {
    private static TrajectorySet INSTANCE;

    public static TrajectorySet getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TrajectorySet();
        }
        return INSTANCE;
    }

    public final Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT;
    public final Trajectory<TimedState<Pose2dWithCurvature>> LIVING_ROOM;
    private TrajectorySet() {
        this.DRIVE_STRAIGHT = new DriveStraight().generateTrajectory();
        this.LIVING_ROOM = new LivingRoomPath().generateTrajectory();
    }
}
