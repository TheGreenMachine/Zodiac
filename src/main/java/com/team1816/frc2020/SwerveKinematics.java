package com.team1816.frc2020;

import com.team1816.frc2020.subsystems.Drive;
import com.team1816.frc2020.subsystems.SwerveModule;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.SwerveDriveSignal;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a swerve drive.
 * <p>
 * Equations extrapolated through papers by Ether, then corrected for our frame of reference:
 * https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
 */

public class SwerveKinematics {

    @Inject
    private static Drive.Factory mDriveFactory;
    private static Translation2d[] moduleRelativePositions = Constants.kModulePositions;
    private static List<Translation2d> moduleRotationDirections = updateRotationDirections();

    private static List<Translation2d> updateRotationDirections() {
        List<Translation2d> directions = new ArrayList<>(moduleRelativePositions.length);
        for (Translation2d position : moduleRelativePositions) {
            directions.add(position.rotateBy(Rotation2d.fromDegrees(90)));
        }
        return directions;
    }

    private static final double L = Constants.kDriveWheelTrackWidthInches;
    private static final double W = Constants.kDriveWheelbaseLengthInches; // Intentional
    private static final double R = Math.hypot(L, W);

    private static Rotation2d[] prev_wheel_azimuths = SwerveDriveSignal.ZERO_AZIMUTH;

    /**
     * Forward kinematics using only encoders
     */
    public static Twist2d forwardKinematics(SwerveDriveSignal drive_signal) {
        return forwardKinematics(
            drive_signal.getWheelSpeeds(),
            drive_signal.getWheelAzimuths()
        );
    }

    /**
     * @param wheel_speeds
     * @param wheel_azimuths
     * @return Twist2d representing forward, strafe, and angular velocities in real world units
     */
    public static Twist2d forwardKinematics(
        double[] wheel_speeds,
        Rotation2d[] wheel_azimuths
    ) {
        double[] vx = new double[4]; // wheel velocities in the x (forward) direction
        double[] vy = new double[4]; // wheel velocities in the y (strafe) direction

        for (int i = 0; i < vx.length; i++) {
            vx[i] = wheel_azimuths[i].cos() * wheel_speeds[i];
            vy[i] = wheel_azimuths[i].sin() * wheel_speeds[i];
        }

        // average possible solutions to minimize error
        double A = (vy[2] + vy[3]) / 2;
        double B = (vy[0] + vy[1]) / 2;
        double C = (vx[0] + vx[3]) / 2;
        double D = (vx[1] + vx[2]) / 2;

        // average possible solutions to minimize error
        double forward = (C + D) / 2;
        double strafe = (A + B) / 2;
        double rotation =
            (
                ((strafe - A) * R / L) +
                ((B - strafe) * R / L) +
                ((forward - C) * R / W) +
                ((D - forward) * R / W)
            ) /
            4;

        return new Twist2d(forward, strafe, rotation);
    }

    /**
     * Use Gyro for dtheta
     */
    public static Twist2d forwardKinematics(
        SwerveDriveSignal drive_signal,
        Rotation2d prev_heading,
        Rotation2d current_heading,
        double dt
    ) {
        Twist2d ret_val = forwardKinematics(drive_signal);
        return new Twist2d(
            ret_val.dx,
            ret_val.dy,
            prev_heading.inverse().rotateBy(current_heading).getRadians() / dt
        );
    }

    public static Twist2d forwardKinematics(
        double[] wheel_speeds,
        Rotation2d[] wheel_azimuths,
        Rotation2d prev_heading,
        Rotation2d current_heading,
        double dt
    ) {
        Twist2d ret_val = forwardKinematics(wheel_speeds, wheel_azimuths);
        return new Twist2d(
            ret_val.dx,
            ret_val.dy,
            prev_heading.inverse().rotateBy(current_heading).getRadians() / dt
        );
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous
     * rotation.
     */
    public static Pose2d integrateForwardKinematics(
        Pose2d current_pose,
        Twist2d forward_kinematics
    ) {
        return current_pose.transformBy(
            new Pose2d(
                forward_kinematics.dx,
                forward_kinematics.dy,
                Rotation2d.fromRadians(forward_kinematics.dtheta)
            )
        );
    }

    public static SwerveDriveSignal inverseKinematics(
        double forward,
        double strafe,
        double rotation,
        boolean field_relative
    ) {
        return inverseKinematics(forward, strafe, rotation, field_relative, true);
    }

    public static SwerveDriveSignal inverseKinematics(
        double forward,
        double strafe,
        double rotation,
        boolean field_relative,
        boolean normalize_outputs
    ) {
        Drive mDrive = mDriveFactory.getInstance();

        if (field_relative) {
            Rotation2d gyroHeading = mDrive.getHeading();
            double temp = forward * gyroHeading.cos() + strafe * gyroHeading.sin();
            strafe = -forward * gyroHeading.sin() + strafe * gyroHeading.cos();
            forward = temp;
        }

        double A = strafe - rotation * L / R; // L / R = cos theta
        double B = strafe + rotation * L / R;
        double C = forward - rotation * W / R; // W / R = sin theta
        double D = forward + rotation * W / R;

        double[] wheel_speeds = new double[4];
        wheel_speeds[SwerveModule.kFrontLeft] = Math.hypot(A, D);
        wheel_speeds[SwerveModule.kFrontRight] = Math.hypot(A, C);
        wheel_speeds[SwerveModule.kBackLeft] = Math.hypot(B, D);
        wheel_speeds[SwerveModule.kBackRight] = Math.hypot(B, C);

        // normalize wheel speeds if above 1
        if (normalize_outputs) {
            double max_speed = 1;
            for (int i = 0; i < wheel_speeds.length; i++) {
                if (Math.abs(wheel_speeds[i]) > max_speed) {
                    max_speed = Math.abs(wheel_speeds[i]);
                }
            }

            for (var i = 0; i < wheel_speeds.length; i++) {
                wheel_speeds[i] /= max_speed;
            }
        }
        Rotation2d[] wheel_azimuths = new Rotation2d[4];

        if (forward != 0 || strafe != 0 || rotation != 0) {
            wheel_azimuths[SwerveModule.kFrontLeft] =
                Rotation2d.fromRadians(Math.atan2(A, D));
            wheel_azimuths[SwerveModule.kFrontRight] =
                Rotation2d.fromRadians(Math.atan2(A, C));
            wheel_azimuths[SwerveModule.kBackLeft] = Rotation2d.fromRadians(Math.atan2(B, D));
            wheel_azimuths[SwerveModule.kBackRight] =
                Rotation2d.fromRadians(Math.atan2(B, C));

            prev_wheel_azimuths = wheel_azimuths;
        } else {
            wheel_azimuths = prev_wheel_azimuths;
        }

        return new SwerveDriveSignal(wheel_speeds, wheel_azimuths, false);
    }

    public static List<Translation2d> updateDriveVectors(
        Translation2d translationalVector,
        double rotationalMagnitude,
        Pose2d robotPose,
        boolean robotCentric
    ) {
        SmartDashboard.putNumber(
            "Drive/Vector Direction",
            translationalVector.direction().getDegrees()
        );
        SmartDashboard.putNumber("Drive/Robot Velocity", translationalVector.norm());

        if (!robotCentric) translationalVector =
            translationalVector.rotateBy(robotPose.getRotation().inverse());
        List<Translation2d> driveVectors = new ArrayList<>(4);
        for (int i = 0; i < 4; i++) {
            var rotationalVector = moduleRotationDirections.get(i).scale(rotationalMagnitude);
            var netVector = translationalVector.translateBy(rotationalVector);
            driveVectors.add(netVector);
        }
        double maxMagnitude = 1.0;
        for (Translation2d t : driveVectors) {
            double magnitude = t.norm();
            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }
        for (int i = 0; i < 4; i++) {
            Translation2d driveVector = driveVectors.get(i);
            driveVectors.set(i, driveVector.scale(1.0 / maxMagnitude));
        }
        return driveVectors;
    }
}
