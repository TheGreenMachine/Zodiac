package com.team1816.lib.subsystems;

import com.team1816.frc2020.Kinematics;
import com.team1816.frc2020.RobotState;
import com.team1816.frc2020.subsystems.Drive;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotStateEstimator extends Subsystem {

    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Turret turret = Turret.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {
        super("RobotStateEstimator");
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {

        @Override
        public synchronized void onStart(double timestamp) {
            left_encoder_prev_distance_ = mDrive.getLeftEncoderDistance();
            right_encoder_prev_distance_ = mDrive.getRightEncoderDistance();
            prev_timestamp_ = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            if (prev_heading_ == null) {
                prev_heading_ = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            }
            final double dt = timestamp - prev_timestamp_;
            final double[] wheel_speeds = mDrive.getModuleVelocities();
            final Rotation2d[] wheel_azimuths = mDrive.getModuleAzimuths();
            final Rotation2d gyro_angle = mDrive.getHeading();
            Twist2d odometry_twist;
            synchronized (mRobotState) {
                final Pose2d last_measurement = mRobotState.getLatestFieldToVehicle().getValue();

                // this should be used for debugging forward kinematics without gyro (shouldn't be used in actual code)
                // odometry_twist = Kinematics.forwardKinematics(wheel_speeds, wheel_azimuths).scaled(dt);

                // this should be used for more accurate measurements for actual code
                odometry_twist = Kinematics.forwardKinematics(wheel_speeds,
                    wheel_azimuths, last_measurement.getRotation(), gyro_angle, dt).scaled(dt);
            }
            final Twist2d measured_velocity = Kinematics.forwardKinematics(
                wheel_speeds, wheel_azimuths, prev_heading_, gyro_angle, dt);
            mRobotState.addObservations(timestamp, odometry_twist, measured_velocity);

            prev_heading_ = gyro_angle;
            prev_timestamp_ = timestamp;
        }

        @Override
        public void onStop(double timestamp) {}
    }

    @Override
    public void stop() {}

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
