package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.frc2020.Constants;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.lib.hardware.PidConfig;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.ISwerveModule;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Util;

import java.util.List;

public class SwerveModule extends Subsystem implements ISwerveModule {

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
        OPEN_LOOP,
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
        public PidConfig kAzimuthPid = PidConfig.EMPTY;
        public int kAzimuthIZone = 25;
        public int kAzimuthCruiseVelocity = 1698;
        public int kAzimuthAcceleration = 20379; // 12 * kAzimuthCruiseVelocity
        public int kAzimuthClosedLoopAllowableError =
            (int) factory.getConstant("drivetrain", "azimuthAllowableErrorTicks");

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
        public VelocityMeasPeriod kAzimuthVelocityMeasurementPeriod =
            VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kAzimuthVelocityMeasurementWindow = 64; // # of samples in rolling average

        // general drive
        public PidConfig kDrivePid = PidConfig.EMPTY;
        public boolean kInvertDrive = true;
        public boolean kInvertDriveSensorPhase = false;
        public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kWheelDiameter = Constants.kDriveWheelDiameterInches; // Probably should tune for each individual wheel maybe
        public double kDriveTicksPerUnitDistance =
            (1.0 / 4096.0) * (18.0 / 28.0 * 15.0 / 45.0) * (Math.PI * kWheelDiameter);
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
        public VelocityMeasPeriod kDriveVelocityMeasurementPeriod =
            VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kDriveVelocityMeasurementWindow = 64; // # of samples in rolling average
    }

    // Components
    private final IMotorControllerEnhanced mDriveMotor;
    private final IMotorControllerEnhanced mAzimuthMotor;

    // State
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;
    private boolean isBrakeMode = false;

    // Constants
    private final SwerveModuleConstants mConstants;

    public static final int kFrontLeft = 0;
    public static final int kFrontRight = 1;
    public static final int kBackLeft = 2;
    public static final int kBackRight = 3;

    public SwerveModule(String subsystemName, SwerveModuleConstants constants) {
        super(constants.kName);
        mConstants = constants;
        System.out.println(
            "Configuring Swerve Module " +
            constants.kName +
            " on subsystem " +
            subsystemName
        );

        mDriveMotor =
            factory.getMotor(
                subsystemName,
                constants.kDriveMotorName,
                List.of(constants.kDrivePid)
            );
        mAzimuthMotor =
            factory.getMotor(
                subsystemName,
                constants.kAzimuthMotorName,
                List.of(constants.kAzimuthPid)
            );

        mAzimuthMotor.setSensorPhase(constants.kInvertAzimuthSensorPhase);
        mAzimuthMotor.configAllowableClosedloopError(0, constants.kAzimuthClosedLoopAllowableError, Constants.kLongCANTimeoutMs);

        System.out.println(mConstants.kName + " drive motor ID: " + mDriveMotor.getDeviceID());
        System.out.println(mConstants.kName + " azimuth motor ID: " + mAzimuthMotor.getDeviceID());

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

        mPeriodicIO.drive_demand = 0/*speed*/;
        mPeriodicIO.azimuth_demand = radiansToEncoderUnits(final_setpoint);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.drive_encoder_ticks = mDriveMotor.getSelectedSensorPosition(0);
        mPeriodicIO.distance =
            (int) encoderUnitsToDistance(mPeriodicIO.drive_encoder_ticks);
        mPeriodicIO.velocity_ticks_per_100ms = mDriveMotor.getSelectedSensorVelocity(0);
        mPeriodicIO.azimuth_encoder_ticks =
            mAzimuthMotor.getSelectedSensorPosition(0) -
            mConstants.kAzimuthEncoderHomeOffset;
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        mDriveMotor.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (
                Util.epsilonEquals(
                    mPeriodicIO.drive_demand,
                    0.0,
                    mConstants.kDriveDeadband
                )
            ) { // don't move if
                // throttle is 0
                stop();
            } else {
                System.out.println("Swerve Module Drive Demand: " + mPeriodicIO.drive_demand);
                mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
            }
        }
        mAzimuthMotor.set(
            ControlMode.Position,
            mPeriodicIO.azimuth_demand + mConstants.kAzimuthEncoderHomeOffset
        );
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(
            new Loop() {
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
                                System.out.println(
                                    "Unexpected control state: " + mControlState
                                );
                                break;
                        }
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    @Override
    public void zeroSensors() {
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        int absolutePosition = getAzimuthPosAbsolute();
        if (mAzimuthMotor instanceof TalonSRX) {
            ((TalonSRX) mAzimuthMotor).getSensorCollection().setQuadraturePosition(
                absolutePosition,
                Constants.kLongCANTimeoutMs
            );
        }
    }

    private int getAzimuthPosAbsolute() {
        if (mAzimuthMotor instanceof TalonSRX) {
            int rawValue =
                ((TalonSRX) mAzimuthMotor).getSensorCollection().getPulseWidthPosition() & 0xFFF;
            return rawValue;
        }
        return 0;
    }

    @Override
    public void stop() {
        mDriveMotor.set(ControlMode.PercentOutput, 0.0);
        mAzimuthMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        boolean driveMotorPassed = EnhancedMotorChecker.checkMotors(
            this,
            EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, mDriveMotor),
            new EnhancedMotorChecker.NamedMotor("drive", mDriveMotor)
        );
        boolean azimuthMotorPassed = EnhancedMotorChecker.checkMotors(
            this,
            EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, mAzimuthMotor),
            new EnhancedMotorChecker.NamedMotor("azimuth", mAzimuthMotor)
        );
        return driveMotorPassed && azimuthMotorPassed;
    }

    @Override
    public double getAzimuthVelocity() {
        return mAzimuthMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public double getAzimuthPosition() {
        return
            (mConstants.kInvertAzimuthSensorPhase ? -1 : 1)
                * ((int) mAzimuthMotor.getSelectedSensorPosition(0) & 0xFFF)
                - mConstants.kAzimuthEncoderHomeOffset;
    }

    @Override
    public double getAzimuthError() {
        return mAzimuthMotor.getClosedLoopError(0);
    }

    @Override
    public double getDriveVelocity() {
        return mDriveMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public double getDriveError() {
        return mDriveMotor.getClosedLoopError(0);
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
        return Util.epsilonEquals(
            mPeriodicIO.azimuth_demand,
            getAngleEncoderUnits(),
            mConstants.kAzimuthClosedLoopAllowableError
        );
    }
}
