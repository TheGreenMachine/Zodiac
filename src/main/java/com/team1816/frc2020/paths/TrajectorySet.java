package com.team1816.frc2020.paths;

import com.team1816.frc2020.paths.paths2020.*;
import com.team1816.frc2020.paths.paths2021.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import javax.inject.Singleton;

@Singleton
public class TrajectorySet {

    private static TrajectorySet INSTANCE;

    // 2020
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> DRIVE_STRAIGHT;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> TUNE_DRIVETRAIN;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> TUNE_DRIVETRAIN_REVERSE;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> LIVING_ROOM;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> SIX_BALL_ALLIANCE;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> AUTO_TRENCH_TURN_RIGHT;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> DRIVE_STRAIGHT_TRENCH;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> DRIVE_STRAIGHT_TRENCH_REVERSE;

    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> FEEDER_TO_TRENCH;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> TRENCH_TO_FEEDER;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> FIVE_BALL_AUTO_OPPOSEA;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> FIVE_BALL_AUTO_OPPOSEB;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> EIGHT_BALL_AUTO_ALLIANCEA;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> EIGHT_BALL_AUTO_ALLIANCEB;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> EIGHT_BALL_AUTO_ALLIANCE_ALTA;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> EIGHT_BALL_AUTO_ALLIANCEC;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> EIGHT_BALL_AUTO_ALLIANCE_ALTB;

    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> EIGHT_BALL_AUTO_OPPOSEA;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> EIGHT_BALL_AUTO_OPPOSEB;
    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> TEN_BALL_AUTO;

    public static Trajectory<TimedState<Pose2dWithCurvature<Pose2d>>> BARREL;

    public TrajectorySet() {
        // 2020
        this.DRIVE_STRAIGHT = new DriveStraight(36).generateTrajectory();
        this.DRIVE_STRAIGHT_TRENCH = new DriveStraight(178, 100).generateTrajectory();
        this.DRIVE_STRAIGHT_TRENCH_REVERSE =
            new DriveStraight(100).generateReversedTrajectory();

        this.TUNE_DRIVETRAIN = new DriveStraight(180, 40).generateTrajectory();
        this.TUNE_DRIVETRAIN_REVERSE = new DriveStraight(-480).generateTrajectory();
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

        this.BARREL = new BarrelPath().generateTrajectory();
    }
}
