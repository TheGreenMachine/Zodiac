package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team1816.frc2020.Constants;
import com.team1816.lib.hardware.MotorUtil;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

public class SwerveModule extends Subsystem {
    public static class PeriodicIO {
        // INPUTS
        public double drive_encoder_ticks;
        public double azimuth_encoder_ticks; // actual position of module in encoder units, adjusted for home offset
        public int position_ticks;
        public int distance;
        public double velocity_ticks_per_100ms;

        // OUTPUTS
        public double drive_demand;
        public double azimuth_demand; // actual desired demand in encoder units, not adjusted for home offset
    }

    public enum ControlState {
        OPEN_LOOP
    }

    public static class SwerveModuleConstants {
        public String kName = "Name";
        public String kDriveMotorName = "";
        public String kAzimuthMotorName = "";

        // general azimuth
        public boolean kInvertAzimuth = false;
        public boolean kInvertAzimuthSensorPhase = false;
        public NeutralMode kAzimuthInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI); // for azimuth
        public double kAzimuthEncoderHomeOffset = 0;

        // azimuth motion
        public double kAzimuthKp = 1.3;
        public double kAzimuthKi = 0.05;
        public double kAzimuthKd = 20;
        public double kAzimuthKf = 0.5421;
        public int kAzimuthIZone = 25;
        public int kAzimuthCruiseVelocity = 1698;
        public int kAzimuthAcceleration = 20379; // 12 * kAzimuthCruiseVelocity
        public int kAzimuthClosedLoopAllowableError = 5;

        // azimuth current/voltage
        public int kAzimuthContinuousCurrentLimit = 30; // amps
        public int kAzimuthPeakCurrentLimit = 60; // amps
        public int kAzimuthPeakCurrentDuration = 200; // ms
        public boolean kAzimuthEnableCurrentLimit = true;
        public double kAzimuthMaxVoltage = 10.0; // volts
        public int kAzimuthVoltageMeasurementFilter = 8; // # of samples in rolling average

        // azimuth measurement
        public int kAzimuthStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
        public int kAzimuthStatusFrame10UpdateRate = 10; // motion magic, ms
        public VelocityMeasPeriod kAzimuthVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kAzimuthVelocityMeasurementWindow = 64; // # of samples in rolling average

        // general drive
        public boolean kInvertDrive = true;
        public boolean kInvertDriveSensorPhase = false;
        public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kWheelDiameter = 4.0; // Probably should tune for each individual wheel maybe
        public double kDriveTicksPerUnitDistance = (1.0 / 4096.0) * (18.0 / 28.0 * 15.0 / 45.0)
            * (Math.PI * kWheelDiameter);
        public double kDriveDeadband = 0.01;

        // drive current/voltage
        public int kDriveContinuousCurrentLimit = 30; // amps
        public int kDrivePeakCurrentLimit = 50; // amps
        public int kDrivePeakCurrentDuration = 200; // ms
        public boolean kDriveEnableCurrentLimit = true;
        public double kDriveMaxVoltage = 10.0; // volts
        public int kDriveVoltageMeasurementFilter = 8; // # of samples in rolling average

        // drive measurement
        public int kDriveStatusFrame2UpdateRate = 15; // feedback for selected sensor, ms
        public int kDriveStatusFrame10UpdateRate = 200; // motion magic, ms
        public VelocityMeasPeriod kDriveVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kDriveVelocityMeasurementWindow = 64; // # of samples in rolling average
    }

    // Components
    private final IMotorControllerEnhanced mDriveMotor;
    private final IMotorControllerEnhanced mAzimuthMotor;

    // State
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;

    // Constants
    private final SwerveModuleConstants mConstants;


    public SwerveModule(String subsystemName, SwerveModuleConstants constants) {
        super(constants.kName);
        mConstants = constants;

        mDriveMotor = factory.getMotor(subsystemName, constants.kDriveMotorName);
        mAzimuthMotor = factory.getMotor(subsystemName, constants.kAzimuthMotorName);

        // config sensors
        MotorUtil.checkError(
            mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive encoder");
        MotorUtil.checkError(
            mAzimuthMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth encoder");

        // config azimuth motion
        MotorUtil.checkError(mAzimuthMotor.config_kP(0, mConstants.kAzimuthKp, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth kp");
        MotorUtil.checkError(mAzimuthMotor.config_kI(0, mConstants.kAzimuthKi, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth ki");
        MotorUtil.checkError(
            mAzimuthMotor.config_IntegralZone(0, mConstants.kAzimuthIZone, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth i zone");
        MotorUtil.checkError(mAzimuthMotor.config_kD(0, mConstants.kAzimuthKd, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth kd");
        MotorUtil.checkError(mAzimuthMotor.config_kF(0, mConstants.kAzimuthKf, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth kf");
        MotorUtil.checkError(
            mAzimuthMotor.configMotionCruiseVelocity(mConstants.kAzimuthCruiseVelocity,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth cruise vel");
        MotorUtil.checkError(
            mAzimuthMotor.configMotionAcceleration(mConstants.kAzimuthAcceleration, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth max acc");
        MotorUtil.checkError(
            mAzimuthMotor.configAllowableClosedloopError(0, mConstants.kAzimuthClosedLoopAllowableError,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth allowable closed loop error");
        mAzimuthMotor.selectProfileSlot(0, 0);

        // config azimuth current/voltage settings
        mAzimuthMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                mConstants.kAzimuthEnableCurrentLimit,
                mConstants.kAzimuthContinuousCurrentLimit,
                mConstants.kAzimuthPeakCurrentLimit,
                mConstants.kAzimuthPeakCurrentDuration
            ),
            Constants.kLongCANTimeoutMs
        );
        MotorUtil.checkError(
            mAzimuthMotor.configVoltageMeasurementFilter(mConstants.kAzimuthVoltageMeasurementFilter,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage measurement filter");
        MotorUtil.checkError(
            mAzimuthMotor.configVoltageCompSaturation(mConstants.kAzimuthMaxVoltage, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage comp saturation");
        mAzimuthMotor.enableVoltageCompensation(true);

        // config azimuth measurement settings
        MotorUtil.checkError(
            mAzimuthMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                mConstants.kAzimuthStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 2 period");
        MotorUtil.checkError(
            mAzimuthMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                mConstants.kAzimuthStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 10 period");
        MotorUtil.checkError(
            mAzimuthMotor.configVelocityMeasurementPeriod(mConstants.kAzimuthVelocityMeasurementPeriod,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement period");
        MotorUtil.checkError(
            mAzimuthMotor.configVelocityMeasurementWindow(mConstants.kAzimuthVelocityMeasurementWindow,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement window");

        // config drive current/voltage settings
        mDriveMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                mConstants.kDriveEnableCurrentLimit,
                mConstants.kDriveContinuousCurrentLimit,
                mConstants.kDrivePeakCurrentLimit,
                mConstants.kDrivePeakCurrentDuration
            ),
            Constants.kLongCANTimeoutMs
        );
        MotorUtil.checkError(
            mDriveMotor.configVoltageMeasurementFilter(mConstants.kDriveVoltageMeasurementFilter,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive voltage measurement filter");
        MotorUtil.checkError(
            mDriveMotor.configVoltageCompSaturation(mConstants.kDriveMaxVoltage, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive voltage comp saturation");
        mDriveMotor.enableVoltageCompensation(true);

        // config drive measurement settings
        MotorUtil.checkError(
            mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                mConstants.kDriveStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive status frame 2 period");
        MotorUtil.checkError(
            mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                mConstants.kDriveStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive status frame 10 period");
        MotorUtil.checkError(
            mDriveMotor.configVelocityMeasurementPeriod(mConstants.kDriveVelocityMeasurementPeriod,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement period");
        MotorUtil.checkError(
            mDriveMotor.configVelocityMeasurementWindow(mConstants.kDriveVelocityMeasurementWindow,
                Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement window");

        // config general drive settings
        mDriveMotor.setInverted(mConstants.kInvertDrive);
        mDriveMotor.setSensorPhase(mConstants.kInvertDriveSensorPhase);
        mDriveMotor.setNeutralMode(mConstants.kDriveInitNeutralMode);
        MotorUtil.checkError(mDriveMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable drive forward soft limit");
        MotorUtil.checkError(mDriveMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable drive reverse soft limit");

        // config general azimuth settings
        mAzimuthMotor.setInverted(mConstants.kInvertAzimuth);
        mAzimuthMotor.setSensorPhase(mConstants.kInvertAzimuthSensorPhase);
        mAzimuthMotor.setNeutralMode(mConstants.kAzimuthInitNeutralMode);
        MotorUtil.checkError(mAzimuthMotor.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable azimuth forward soft limit");
        MotorUtil.checkError(mAzimuthMotor.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
            "Error in " + mConstants.kName + "Module: Unable to disable azimuth reverse soft limit");

        zeroSensors();
    }

    public synchronized void setOpenLoop(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        Rotation2d current = getAngle();

        double raw_error = current.distance(azimuth);
        if (Math.abs(raw_error) > Math.PI) {
            raw_error -= (Math.PI * 2 * Math.signum(raw_error));
        }

        // error is -180 to 180
        // is wheel reversible logic
        if (Math.abs(raw_error) > Math.PI / 2) {
            speed *= -1;
            raw_error -= Math.PI * Math.signum(raw_error);
        }

        double final_setpoint = getRawAngle() + raw_error;
        // double adjusted_speed = speed * Math.abs(Math.cos(raw_error));

        mPeriodicIO.drive_demand = speed;
        mPeriodicIO.azimuth_demand = radiansToEncoderUnits(final_setpoint);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.drive_encoder_ticks = mDriveMotor.getSelectedSensorPosition(0);
        mPeriodicIO.distance = (int) encoderUnitsToDistance(mPeriodicIO.drive_encoder_ticks);
        mPeriodicIO.velocity_ticks_per_100ms = mDriveMotor.getSelectedSensorVelocity(0);
        mPeriodicIO.azimuth_encoder_ticks = mAzimuthMotor.getSelectedSensorPosition(0)
            - mConstants.kAzimuthEncoderHomeOffset;
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (Util.epsilonEquals(mPeriodicIO.drive_demand, 0.0, mConstants.kDriveDeadband)) { // don't move if
                // throttle is 0
                stop();
            } else {
                mAzimuthMotor.set(ControlMode.MotionMagic,
                    mPeriodicIO.azimuth_demand + mConstants.kAzimuthEncoderHomeOffset);
                mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
            }
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (SwerveModule.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (SwerveModule.this) {
                    switch (mControlState) {
                        case OPEN_LOOP:
                            break;
                        default:
                            System.out.println("Unexpected control state: " + mControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void zeroSensors() {
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        /* Azimuth Talon should be in absolute mode */
    }

    @Override
    public void stop() {
        mDriveMotor.set(ControlMode.PercentOutput, 0.0);
        mAzimuthMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * @param ticks azimuth ticks
     */
    public synchronized double encoderUnitsToRadians(double ticks) {
        return ticks / mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @return azimuth ticks
     */
    public synchronized double radiansToEncoderUnits(double radians) {
        return radians * mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @param ticks drive ticks
     */
    public synchronized double encoderUnitsToDistance(double ticks) {
        return ticks * mConstants.kDriveTicksPerUnitDistance;
    }

    /**
     * @return drive ticks
     */
    public synchronized double distanceToEncoderUnits(double distance) {
        return distance / mConstants.kDriveTicksPerUnitDistance;
    }

    public synchronized double getAngleEncoderUnits() {
        return mPeriodicIO.azimuth_encoder_ticks;
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians((encoderUnitsToRadians(getAngleEncoderUnits())));
    }

    public synchronized double getRawAngle() {
        return encoderUnitsToRadians(getAngleEncoderUnits());
    }

    public synchronized double getUnwrappedAngleDegrees() {
        return Math.toDegrees(encoderUnitsToRadians(getAngleEncoderUnits()));
    }

    public synchronized double getRawLinearVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms * 10;
    }

    public synchronized double getLinearVelocity() {
        return encoderUnitsToDistance(getRawLinearVelocity());
    }

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        mDriveMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void setAzimuthBrakeMode(boolean brake_mode) {
        mAzimuthMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized double getDrivePercentOutput() {
        return mDriveMotor.getMotorOutputPercent();
    }

    public synchronized boolean isAzimuthAtTarget() {
        return Util.epsilonEquals(mPeriodicIO.azimuth_demand, getAngleEncoderUnits(),
            mConstants.kAzimuthClosedLoopAllowableError);
    }
}
