package com.team1816.frc2020.paths;

import com.team1816.frc2020.paths.paths2020.*;
import com.team1816.frc2020.paths.paths2021.*;
import com.team1816.frc2020.subsystems.LedManager;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

public class TrajectorySet {

    private static TrajectorySet INSTANCE;

    public static TrajectorySet getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TrajectorySet();
        }
        return INSTANCE;
    }

    // 2020
    public Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT;
    public Trajectory<TimedState<Pose2dWithCurvature>> TUNE_DRIVETRAIN;
    public Trajectory<TimedState<Pose2dWithCurvature>> TUNE_DRIVETRAIN_REVERSE;
    public Trajectory<TimedState<Pose2dWithCurvature>> LIVING_ROOM;
    public Trajectory<TimedState<Pose2dWithCurvature>> SIX_BALL_ALLIANCE;
    public Trajectory<TimedState<Pose2dWithCurvature>> AUTO_TRENCH_TURN_RIGHT;
    public Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT_TRENCH;
    public Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT_TRENCH_REVERSE;

    public Trajectory<TimedState<Pose2dWithCurvature>> FEEDER_TO_TRENCH;
    public Trajectory<TimedState<Pose2dWithCurvature>> TRENCH_TO_FEEDER;
    public Trajectory<TimedState<Pose2dWithCurvature>> FIVE_BALL_AUTO_OPPOSEA;
    public Trajectory<TimedState<Pose2dWithCurvature>> FIVE_BALL_AUTO_OPPOSEB;
    public Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEA;
    public Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEB;
    public Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCE_ALTA;
    public Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEC;
    public Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCE_ALTB;

    public Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEA;
    public Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEB;
    public Trajectory<TimedState<Pose2dWithCurvature>> TEN_BALL_AUTO;

    private TrajectorySet() {
        // 2020
        var future = CompletableFuture.allOf(
            new DriveStraight(12).generateTrajectory()
                .thenAccept(trajectory -> this.DRIVE_STRAIGHT = trajectory),

            new DriveStraight(178, 100).generateTrajectory()
                .thenAccept(trajectory -> this.DRIVE_STRAIGHT_TRENCH = trajectory),

            new DriveStraight(100).generateReversedTrajectory()
                .thenAccept(trajectory -> this.DRIVE_STRAIGHT_TRENCH_REVERSE = trajectory),

            new DriveStraight(180, 40).generateTrajectory()
                .thenAccept(trajectory -> this.TUNE_DRIVETRAIN = trajectory),

            new DriveStraight(-480).generateTrajectory()
                .thenAccept(trajectory -> this.TUNE_DRIVETRAIN_REVERSE = trajectory),

            new LivingRoomPath().generateTrajectory()
                .thenAccept(trajectory -> this.LIVING_ROOM = trajectory),

            new AutoTrenchTurnRightPath().generateTrajectory()
                .thenAccept(trajectory -> this.AUTO_TRENCH_TURN_RIGHT = trajectory),

            new SixBallAlliancePath().generateTrajectory()
                .thenAccept(trajectory -> this.SIX_BALL_ALLIANCE = trajectory),

            new FeederToTrenchPath().generateTrajectory()
                .thenAccept(trajectory -> this.FEEDER_TO_TRENCH = trajectory),
            new FeederToTrenchPath().generateReversedTrajectory()
                .thenAccept(trajectory -> this.TRENCH_TO_FEEDER = trajectory),

            new DriveStraight(96).generateTrajectory()
                .thenAccept(trajectory -> this.FIVE_BALL_AUTO_OPPOSEA = trajectory),
            new FiveBallOpposingPathPartB().generateTrajectory()
                .thenAccept(trajectory -> this.FIVE_BALL_AUTO_OPPOSEB = trajectory),

            new DriveStraight(110).generateReversedTrajectory()
                .thenAccept(trajectory -> this.EIGHT_BALL_AUTO_ALLIANCEA = trajectory),

            new EightBallAllianceAltAPath().generateTrajectory()
                .thenAccept(trajectory -> this.EIGHT_BALL_AUTO_ALLIANCE_ALTA = trajectory),

            new EightBallAllianceBPath().generateTrajectory()
                .thenAccept(trajectory -> this.EIGHT_BALL_AUTO_ALLIANCEB = trajectory),

            new DriveStraight(20).generateTrajectory()
                .thenAccept(trajectory -> this.EIGHT_BALL_AUTO_ALLIANCE_ALTB = trajectory),

            new EightBallAllianceCPath().generateTrajectory()
                .thenAccept(trajectory -> this.EIGHT_BALL_AUTO_ALLIANCEC = trajectory),

            new EightBallOpposingAPath().generateTrajectory()
                .thenAccept(trajectory -> this.EIGHT_BALL_AUTO_OPPOSEA = trajectory),

            new EightBallOpposingBPath().generateTrajectory()
                .thenAccept(trajectory -> this.EIGHT_BALL_AUTO_OPPOSEB = trajectory),

            new TenBallOpposingPath().generateTrajectory()
                .thenAccept(trajectory -> this.TEN_BALL_AUTO = trajectory)
        );

        try {
            future.get();
            System.out.println("Trajectory set calculated");
        } catch (InterruptedException | ExecutionException e) {
            DriverStation.reportError("Error generating trajectories!", e.getStackTrace());
            LedManager.getInstance().indicateStatus(LedManager.RobotStatus.ERROR);
        }
    }
}
