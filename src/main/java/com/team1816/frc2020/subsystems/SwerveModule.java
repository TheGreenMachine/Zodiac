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
import edu.wpi.first.wpilibj.RobotBase;

import java.util.List;

public class SwerveModule extends Subsystem implements ISwerveModule {

    public static class PeriodicIO {

        // INPUTS
        public double drive_encoder_ticks;
        public double azimuth_encoder_ticks; // actual position of module in encoder units, adjusted for home offset
        public int position_ticks;
        public double velocity_ticks_per_100ms;

        // OUTPUTS
        public double drive_demand;
        public double azimuth_demand; // actual desired demand in encoder units, not adjusted for home offset
    }

    public enum ControlState {
        OPEN_LOOP, VELOCITY
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
        public int kAzimuthClosedLoopAllowableError = (int) factory.getConstant(
            "drivetrain",
            "azimuthAllowableErrorTicks"
        );

        // azimuth current/voltage
        public VelocityMeasPeriod kAzimuthVelocityMeasurementPeriod =
            VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms

        // general drive
        public PidConfig kDrivePid = PidConfig.EMPTY;
        public double kWheelDiameter = Constants.kDriveWheelDiameterInches; // Probably should tune for each individual wheel maybe
        public double kDriveTicksPerUnitDistance =
            (1 / Drive.DRIVE_ENCODER_PPR) * (Math.PI * kWheelDiameter);
        public double kDriveDeadband = 0.01;

        // drive current/voltage

        // drive measurement
        public VelocityMeasPeriod kDriveVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
    }

    // Components
    private final IMotorControllerEnhanced mDriveMotor;
    private final IMotorControllerEnhanced mAzimuthMotor;

    // State
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;
    private boolean isBrakeMode = false;
    private double driveEncoderSimPosition = 0;

    // Constants
    private final SwerveModuleConstants mConstants;
    private static final double TICK_RATIO_PER_LOOP = Constants.kLooperDt / 0.1;

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
        mAzimuthMotor.setNeutralMode(NeutralMode.Brake);
        mAzimuthMotor.configAllowableClosedloopError(
            0,
            constants.kAzimuthClosedLoopAllowableError,
            Constants.kLongCANTimeoutMs
        );
        System.out.println("  " + this);
        zeroSensors();
    }

    public synchronized void setOpenLoop(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }


        mPeriodicIO.drive_demand = speed;
        mPeriodicIO.azimuth_demand = (int) radiansToEncoderUnits(azimuth.getRadians());
    }

    public synchronized void setVelocity(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.VELOCITY) {
            mControlState = ControlState.VELOCITY;
        }

        mPeriodicIO.drive_demand = speed;
        mPeriodicIO.azimuth_demand = (int) radiansToEncoderUnits(azimuth.getRadians());
    }

    @Override
    public void readPeriodicInputs() {
        if (RobotBase.isSimulation()) {
            driveEncoderSimPosition += mPeriodicIO.drive_demand * TICK_RATIO_PER_LOOP;
            mPeriodicIO.drive_encoder_ticks = driveEncoderSimPosition;
            mPeriodicIO.velocity_ticks_per_100ms = mPeriodicIO.drive_demand;
            mPeriodicIO.azimuth_encoder_ticks = mPeriodicIO.azimuth_demand;

        } else {
            mPeriodicIO.drive_encoder_ticks = mDriveMotor.getSelectedSensorPosition(0);
            mPeriodicIO.velocity_ticks_per_100ms = mDriveMotor.getSelectedSensorVelocity(0);
            mPeriodicIO.azimuth_encoder_ticks =
                (mConstants.kInvertAzimuthSensorPhase ? -1 : 1) *
                    ((int) mAzimuthMotor.getSelectedSensorPosition(0) & 0xFFF) -
                    mConstants.kAzimuthEncoderHomeOffset;
        }
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
                System.out.println(mConstants.kName + " drive demand: " + mPeriodicIO.drive_demand);
                mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
            }
        } else if (mControlState == ControlState.VELOCITY) {
//            System.out.println(mConstants.kName + " drive demand: " + mPeriodicIO.drive_demand);
            mDriveMotor.set(ControlMode.Velocity, mPeriodicIO.drive_demand);
        }
        var offsetDemand = ((int) (mPeriodicIO.azimuth_demand + mConstants.kAzimuthEncoderHomeOffset)) & 0xFFF;
        mAzimuthMotor.set(ControlMode.Position, offsetDemand);
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
                            case VELOCITY:
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
        if (mAzimuthMotor instanceof TalonSRX) {
            var sensors = ((TalonSRX) mAzimuthMotor).getSensorCollection();
            sensors.setQuadraturePosition(
                sensors.getPulseWidthPosition() & 0xFFF,
                Constants.kLongCANTimeoutMs
            );
        }
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
    }

    private int getAzimuthPosAbsolute() {
        if (mAzimuthMotor instanceof TalonSRX) {
            int rawValue =
                ((TalonSRX) mAzimuthMotor).getSensorCollection().getPulseWidthPosition() &
                    0xFFF;
            return rawValue;
        }
        return 0;
    }

    @Override
    public void stop() {
        mDriveMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        boolean driveMotorPassed = EnhancedMotorChecker.checkMotors(
            this,
            EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, mDriveMotor),
            new EnhancedMotorChecker.NamedMotor(mConstants.kDriveMotorName, mDriveMotor)
        );
        boolean azimuthMotorPassed = EnhancedMotorChecker.checkMotors(
            this,
            EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, mAzimuthMotor),
            new EnhancedMotorChecker.NamedMotor(
                mConstants.kAzimuthMotorName,
                mAzimuthMotor
            )
        );
        zeroSensors();
        return driveMotorPassed && azimuthMotorPassed;
    }

    @Override
    public double getAzimuthVelocity() {
        return mAzimuthMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public double getAzimuthPosition() {
        return mPeriodicIO.azimuth_encoder_ticks;
    }

    @Override
    public double getAzimuthError() {
        return mAzimuthMotor.getClosedLoopError(0);
    }

    @Override
    public double getDriveVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms;
    }

    @Override
    public double getDriveVelocityDemand() {
        return mPeriodicIO.drive_demand;
    }

    @Override
    public double getDriveError() {
        return mDriveMotor.getClosedLoopError(0);
    }

    @Override
    public double getDriveDistance() {
        return Drive.rotationsToInches(mDriveMotor.getSelectedSensorPosition(0) / Drive.DRIVE_ENCODER_PPR);
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

    @Override
    public String toString() {
        return (
            "SwerveModule{ " + mConstants.kDriveMotorName +
            " id: " + mDriveMotor.getDeviceID() +
            "  " + mConstants.kAzimuthMotorName +
            " id: " + mAzimuthMotor.getDeviceID() +
            " offset: " + mConstants.kAzimuthEncoderHomeOffset +
            " invertSensor: " + mConstants.kInvertAzimuthSensorPhase +
            " invertAzimuth: " + mConstants.kInvertAzimuth +
            " encPPR: " + Drive.DRIVE_ENCODER_PPR +
            " }"
        );
    }
}
