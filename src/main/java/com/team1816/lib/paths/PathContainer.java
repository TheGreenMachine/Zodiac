package com.team1816.lib.paths;

import com.team1816.frc2020.Robot;
import com.team1816.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.Arrays;
import java.util.List;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {

    // velocities are in/sec
    double kMaxVelocity = Robot.getFactory().getConstant("maxVel");
    double kMaxAccel = Robot.getFactory().getConstant("maxAccel");
    double kMaxCentripetalAccel = 100.0;
    double kMaxVoltage = 9.0;

    Path buildPath();

    List<Pose2d> buildWaypoints();

    default Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory() {
        return DriveMotionPlanner.getInstance().generateTrajectory(
            false,
            buildWaypoints(),
            Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity, kMaxAccel, kMaxVoltage);
    }

    default Trajectory.Mirrored generateMirroredTrajectory() {
        return new Trajectory.Mirrored(generateTrajectory());
    }


    boolean isReversed();
}
