package com.team1816.frc2020.paths;

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
    public final Trajectory<TimedState<Pose2dWithCurvature>> AUTO_TRENCH;

    public final Trajectory<TimedState<Pose2dWithCurvature>> FEEDER_TO_TRENCH;
    public final Trajectory<TimedState<Pose2dWithCurvature>> TRENCH_TO_FEEDER;

    private TrajectorySet() {
        this.DRIVE_STRAIGHT = new DriveStraight().generateTrajectory();
        this.LIVING_ROOM = new LivingRoomPath().generateTrajectory();
        this.AUTO_TRENCH = new AutoTrenchPath().generateTrajectory();
        this.FEEDER_TO_TRENCH = new FeederToTrenchPath().generateTrajectory();
        this.TRENCH_TO_FEEDER = new FeederToTrenchPath().generateMirroredTrajectory();
    }
}
