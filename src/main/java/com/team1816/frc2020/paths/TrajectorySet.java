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
    public final Trajectory<TimedState<Pose2dWithCurvature>> SIX_BALL_ALLIANCE;
    public final Trajectory<TimedState<Pose2dWithCurvature>> AUTO_TRENCH_TURN_RIGHT;

    public final Trajectory<TimedState<Pose2dWithCurvature>> FEEDER_TO_TRENCH;
    public final Trajectory<TimedState<Pose2dWithCurvature>> TRENCH_TO_FEEDER;
    public final Trajectory<TimedState<Pose2dWithCurvature>> FIVE_BALL_AUTO_OPPOSE;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCE;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEA;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEB;
    public final Trajectory<TimedState<Pose2dWithCurvature>> TEN_BALL_AUTO;

    private TrajectorySet() {
        this.DRIVE_STRAIGHT = new DriveStraight().generateTrajectory();
        this.LIVING_ROOM = new LivingRoomPath().generateTrajectory();
        this.SIX_BALL_ALLIANCE = new SixBallAlliancePath().generateTrajectory();
        this.AUTO_TRENCH_TURN_RIGHT = new AutoTrenchTurnRightPath().generateTrajectory();
        this.FEEDER_TO_TRENCH = new FeederToTrenchPath().generateTrajectory();
        this.TRENCH_TO_FEEDER = new FeederToTrenchPath().generateReversedTrajectory();
        this.FIVE_BALL_AUTO_OPPOSE = new FiveBallOpposingPath().generateTrajectory();
        this.EIGHT_BALL_AUTO_ALLIANCE = new EightBallAlliancePath().generateTrajectory();
        this.EIGHT_BALL_AUTO_OPPOSEA = new EightBallOpposingAPath().generateTrajectory();
        this.EIGHT_BALL_AUTO_OPPOSEB = new EightBallOpposingBPath().generateTrajectory();
        this.TEN_BALL_AUTO = new TenBallOpposingPath().generateTrajectory();
    }
}
