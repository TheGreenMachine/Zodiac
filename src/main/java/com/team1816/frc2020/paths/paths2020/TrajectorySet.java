package com.team1816.frc2020.paths.paths2020;

import com.team1816.frc2020.paths.paths2021.BlueRedPathB;
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

    // 2020
    public final Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT;
    public final Trajectory<TimedState<Pose2dWithCurvature>> TUNE_DRIVETRAIN;
    public final Trajectory<TimedState<Pose2dWithCurvature>> LIVING_ROOM;
    public final Trajectory<TimedState<Pose2dWithCurvature>> SIX_BALL_ALLIANCE;
    public final Trajectory<TimedState<Pose2dWithCurvature>> AUTO_TRENCH_TURN_RIGHT;
    public final Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT_TRENCH;
    public final Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT_TRENCH_REVERSE;

    public final Trajectory<TimedState<Pose2dWithCurvature>> FEEDER_TO_TRENCH;
    public final Trajectory<TimedState<Pose2dWithCurvature>> TRENCH_TO_FEEDER;
    public final Trajectory<TimedState<Pose2dWithCurvature>> FIVE_BALL_AUTO_OPPOSEA;
    public final Trajectory<TimedState<Pose2dWithCurvature>> FIVE_BALL_AUTO_OPPOSEB;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEA;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEB;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCE_ALTA;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEC;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCE_ALTB;

    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEA;
    public final Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEB;
    public final Trajectory<TimedState<Pose2dWithCurvature>> TEN_BALL_AUTO;

    // 2021

    public final Trajectory<TimedState<Pose2dWithCurvature>> BLUE_RED_PATHB;

    private TrajectorySet() {
        // 2020
        this.DRIVE_STRAIGHT = new DriveStraight(12).generateTrajectory();
        this.DRIVE_STRAIGHT_TRENCH = new DriveStraight(178, 100).generateTrajectory();
        this.DRIVE_STRAIGHT_TRENCH_REVERSE =
            new DriveStraight(100).generateReversedTrajectory();

        this.TUNE_DRIVETRAIN = new DriveStraight(155).generateTrajectory();
        this.LIVING_ROOM = new LivingRoomPath().generateTrajectory();
        this.AUTO_TRENCH_TURN_RIGHT = new AutoTrenchTurnRightPath().generateTrajectory();

        this.SIX_BALL_ALLIANCE = new SixBallAlliancePath().generateTrajectory();

        this.FEEDER_TO_TRENCH = new FeederToTrenchPath().generateTrajectory();
        this.TRENCH_TO_FEEDER = new FeederToTrenchPath().generateReversedTrajectory();

        this.FIVE_BALL_AUTO_OPPOSEA = new DriveStraight(96).generateTrajectory();
        this.FIVE_BALL_AUTO_OPPOSEB =
            new FiveBallOpposingPathPartB().generateTrajectory();

        this.EIGHT_BALL_AUTO_ALLIANCEA =
            new DriveStraight(110).generateReversedTrajectory();
        this.EIGHT_BALL_AUTO_ALLIANCE_ALTA =
            new EightBallAllianceAltAPath().generateTrajectory();
        this.EIGHT_BALL_AUTO_ALLIANCEB =
            new EightBallAllianceBPath().generateTrajectory();
        this.EIGHT_BALL_AUTO_ALLIANCE_ALTB = new DriveStraight(20).generateTrajectory();
        this.EIGHT_BALL_AUTO_ALLIANCEC =
            new EightBallAllianceCPath().generateTrajectory();

        this.EIGHT_BALL_AUTO_OPPOSEA = new EightBallOpposingAPath().generateTrajectory();
        this.EIGHT_BALL_AUTO_OPPOSEB = new EightBallOpposingBPath().generateTrajectory();

        this.TEN_BALL_AUTO = new TenBallOpposingPath().generateTrajectory();

        // 2021

        this.BLUE_RED_PATHB = new BlueRedPathB().generateTrajectory();
    }
}
