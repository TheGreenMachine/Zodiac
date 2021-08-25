package com.team1816.lib.subsystems;

import com.team1816.frc2020.SwerveKinematics;
import com.team1816.frc2020.RobotState;
import com.team1816.frc2020.WestCoastKinematics;
import com.team1816.frc2020.subsystems.Drive;
import com.team1816.frc2020.subsystems.SwerveDrive;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.frc2020.subsystems.WestCoastDrive;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Inject;

public class RobotStateEstimator extends Subsystem {

    @Inject
    private Drive.Factory mDriveFactory;
    private final RobotState mRobotState = RobotState.getInstance();
    private final Turret turret = Turret.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;

    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    public RobotStateEstimator() {
        super("RobotStateEstimator");
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {

        @Override
        public synchronized void onStart(double timestamp) {
            prev_timestamp_ = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            Drive mDrive = mDriveFactory.getInstance();
            if (prev_heading_ == null) {
                prev_heading_ =
                    mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            }
            final double dt = timestamp - prev_timestamp_;
            final Rotation2d gyro_angle = mDrive.getHeading();

            if (mDrive instanceof SwerveDrive) {
                estimateSwerve((SwerveDrive) mDrive, timestamp, dt, gyro_angle);
            } else if (mDrive instanceof WestCoastDrive) {
                estimateWestCoast((WestCoastDrive) mDrive, timestamp, dt);
            } else {
                DriverStation.reportError("RobotStateEstimator - Drive is not of known type", false);
            }

            prev_heading_ = gyro_angle;
            prev_timestamp_ = timestamp;
        }

        @Override
        public void onStop(double timestamp) {
        }
    }

    private void estimateWestCoast(WestCoastDrive mDrive, double timestamp, double dt) {
        final double left_distance = mDrive.getLeftEncoderDistance();
        final double right_distance = mDrive.getRightEncoderDistance();
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final Rotation2d gyro_angle = mDrive.getHeading();
        final Rotation2d gyro_angle_relative_to_initial = mDrive.getHeadingRelativeToInitial();

        Twist2d odometry_twist;
        /* final */Rotation2d turret_angle = Rotation2d.fromDegrees(
            turret.getActualTurretPositionDegrees() - Turret.CARDINAL_SOUTH
        ); // - Turret.CARDINAL_NORTH);
        synchronized (mRobotState) {
            final Pose2d last_measurement = mRobotState
                .getLatestFieldToVehicle()
                .getValue();
            odometry_twist =
                WestCoastKinematics.forwardKinematics(
                    last_measurement.getRotation(),
                    delta_left,
                    delta_right,
                    gyro_angle
                );
        }
        final Twist2d measured_velocity = WestCoastKinematics
            .forwardKinematics(
                delta_left,
                delta_right,
                prev_heading_.inverse().rotateBy(gyro_angle).getRadians()
            )
            .scaled(1.0 / dt);
        final Twist2d predicted_velocity = WestCoastKinematics
            .forwardKinematics(
                mDrive.getLeftLinearVelocity(),
                mDrive.getRightLinearVelocity()
            )
            .scaled(dt);
        mRobotState.addObservations(
            timestamp,
            odometry_twist,
            measured_velocity,
            predicted_velocity
        );
        mRobotState.setHeadingRelativeToInitial(gyro_angle_relative_to_initial);
        mRobotState.addVehicleToTurretObservation(timestamp, turret_angle);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

    private void estimateSwerve(SwerveDrive mDrive, double timestamp, double dt, Rotation2d gyro_angle) {
        final double[] wheel_speeds = mDrive.getModuleVelocities();
        final Rotation2d[] wheel_azimuths = mDrive.getModuleAzimuths();

        Twist2d odometry_twist;
        synchronized (mRobotState) {
            final Pose2d last_measurement = mRobotState
                .getLatestFieldToVehicle()
                .getValue();

            // this should be used for debugging forward kinematics without gyro (shouldn't be used in actual code)
            // odometry_twist = Kinematics.forwardKinematics(wheel_speeds, wheel_azimuths).scaled(dt);

            // this should be used for more accurate measurements for actual code
            odometry_twist =
                SwerveKinematics
                    .forwardKinematics(
                        wheel_speeds,
                        wheel_azimuths,
                        last_measurement.getRotation(),
                        gyro_angle,
                        dt
                    )
                    .scaled(dt);
        }
        final Twist2d measured_velocity = SwerveKinematics.forwardKinematics(
            wheel_speeds,
            wheel_azimuths,
            prev_heading_,
            gyro_angle,
            dt
        );
        mRobotState.addFieldToVehicleObservation(timestamp, mDrive.getPose());
        mRobotState.setHeadingRelativeToInitial(mDrive.getHeadingRelativeToInitial());
        //    mRobotState.addObservations(timestamp, odometry_twist, measured_velocity);
    }


    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mRobotState.outputToSmartDashboard();
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putNumber(
            "turret angle",
            (turret.getTurretPositionDegrees() - Turret.CARDINAL_SOUTH)
        ); // - Turret.CARDINAL_NORTH);
        SmartDashboard.putNumber(
            "test num",
            Rotation2d.fromDegrees(turret.getTurretPositionDegrees()).getDegrees()
        );
    }
}
