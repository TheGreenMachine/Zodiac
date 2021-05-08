package com.team1816.lib.subsystems;

import com.team1816.frc2020.RobotState;
import com.team1816.frc2020.subsystems.Drive;
import com.team1816.frc2020.subsystems.SwerveModule;
import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RobotStateEstimator extends Subsystem {

    private static RobotStateEstimator INSTANCE = new RobotStateEstimator();

    private final RobotState mRobotState = RobotState.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final Turret turret = Turret.getInstance();

    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    public static RobotStateEstimator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotStateEstimator();
        }

        return INSTANCE;
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
            prev_timestamp_ = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            if (prev_heading_ == null) {
                prev_heading_ =
                    mRobotState.getLatestFieldToVehicle().getValue().getRotation();
            }
            final Rotation2d gyro_angle = mDrive.getHeading();
            Pose2d updatedPose;
            synchronized (mRobotState) {
                final Pose2d last_measurement = mRobotState
                    .getLatestFieldToVehicle()
                    .getValue();

                // this should be used for debugging forward kinematics without gyro (shouldn't be used in actual code)
                // odometry_twist = Kinematics.forwardKinematics(wheel_speeds, wheel_azimuths).scaled(dt);

                // this should be used for more accurate measurements for actual code
                updatedPose =
                    updateDrivetrainPose(timestamp, last_measurement, gyro_angle);
                // updatedPose = alternatePoseUpdate(timestamp, last_measurement, gyro_angle);
            }
            mRobotState.addFieldToVehicleObservation(timestamp, updatedPose);
            //    mRobotState.addObservations(timestamp, odometry_twist, measured_velocity);

            prev_heading_ = gyro_angle;
            prev_timestamp_ = timestamp;
        }

        @Override
        public void onStop(double timestamp) {}
    }

    /** The tried and true algorithm for keeping track of position */
    public synchronized Pose2d updateDrivetrainPose(
        double timestamp,
        Pose2d last_measurement,
        Rotation2d heading
    ) {
        double x = 0.0;
        double y = 0.0;

        double averageDistance = 0.0;
        double[] distances = new double[4];
        var swerveModules = mDrive.getSwerveModules();
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule m = swerveModules[i];
            m.updatePose(heading);
            double distance = m
                .getEstimatedRobotPose()
                .getTranslation()
                .translateBy(last_measurement.getTranslation().inverse())
                .norm();
            distances[i] = distance;
            averageDistance += distance;
        }
        averageDistance /= swerveModules.length;

        int minDevianceIndex = 0;
        double minDeviance = 100.0;
        List<SwerveModule> modulesToUse = new ArrayList<>();
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule m = swerveModules[i];
            double deviance = Math.abs(distances[i] - averageDistance);
            if (deviance < minDeviance) {
                minDeviance = deviance;
                minDevianceIndex = i;
            }
            if (deviance <= 0.01) {
                modulesToUse.add(m);
            }
        }

        if (modulesToUse.isEmpty()) {
            modulesToUse.add(swerveModules[minDevianceIndex]);
        }

        //SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for (SwerveModule m : modulesToUse) {
            x += m.getEstimatedRobotPose().getTranslation().x();
            y += m.getEstimatedRobotPose().getTranslation().y();
        }

        Pose2d updatedPose = new Pose2d(
            new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()),
            heading
        );
        double deltaPos = updatedPose
            .getTranslation()
            .translateBy(last_measurement.getTranslation().inverse())
            .norm();
        mDrive.updateOdometry(deltaPos, deltaPos / (timestamp - prev_timestamp_));
        for (SwerveModule module : swerveModules) {
            module.resetPose(updatedPose);
        }
        return updatedPose;
    }

    public synchronized Pose2d alternatePoseUpdate(
        double timestamp,
        Pose2d last_measurement,
        Rotation2d heading
    ) {
        double x = 0.0;
        double y = 0.0;

        double[] distances = new double[4];

        var swerveModules = mDrive.getSwerveModules();
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].updatePose(heading);
            double distance =
                swerveModules[i].getEstimatedRobotPose()
                    .getTranslation()
                    .distance(last_measurement.getTranslation());
            distances[i] = distance;
        }

        Arrays.sort(distances); // Doing some kind of sort for some reason, not sure why

        List<SwerveModule> modulesToUse = new ArrayList<>();
        double firstDifference = distances[1] - distances[0];
        double secondDifference = distances[2] - distances[1];
        double thirdDifference = distances[3] - distances[2];

        if (secondDifference > (1.5 * firstDifference)) {
            modulesToUse.add(swerveModules[0]);
            modulesToUse.add(swerveModules[1]);
        } else if (thirdDifference > (1.5 * firstDifference)) {
            modulesToUse.add(swerveModules[0]);
            modulesToUse.add(swerveModules[1]);
            modulesToUse.add(swerveModules[2]);
        } else {
            modulesToUse.add(swerveModules[0]);
            modulesToUse.add(swerveModules[1]);
            modulesToUse.add(swerveModules[2]);
            modulesToUse.add(swerveModules[3]);
        }

        SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for (SwerveModule m : modulesToUse) {
            x += m.getEstimatedRobotPose().getTranslation().x();
            y += m.getEstimatedRobotPose().getTranslation().y();
        }

        Pose2d updatedPose = new Pose2d(
            new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()),
            heading
        );
        double deltaPos = updatedPose
            .getTranslation()
            .distance(last_measurement.getTranslation());
        mDrive.updateOdometry(deltaPos, deltaPos / (timestamp - prev_timestamp_));

        for (SwerveModule mModule : swerveModules) {
            mModule.resetPose(updatedPose);
        }

        return updatedPose;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
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
