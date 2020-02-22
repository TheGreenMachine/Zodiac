package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.frc2020.Constants;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends Subsystem implements PidProvider {
    private static final String NAME = "turret";
    private static Turret INSTANCE;

    public static Turret getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Turret();
        }
        return INSTANCE;
    }

    // Components
    private final IMotorControllerEnhanced turret;
    private final NetworkTable networkTable;

    // State
    private double turretPos;
    private double turretSpeed;
    private boolean outputsChanged;
    private boolean isPercentOutput;
    private double deltaXAngle;
    private boolean autoHomeEnabled;

    // Constants
    private static final int kPIDLoopIDx = 0;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private static final double TURRET_ENCODER_PPR = factory.getConstant("turret", "encPPR");
    private static final int ALLOWABLE_ERROR_TICKS = 5;
    private static final double TURRET_JOG_DEGREES = 10;
    private static final double TURRET_JOG_TICKS = convertTurretDegreesToTicks(TURRET_JOG_DEGREES);
    public static final int TURRET_POSITION_MIN = ((int) factory.getConstant("turret", "minPos"));
    public static final int TURRET_POSITION_MAX = ((int) factory.getConstant("turret", "maxPos"));
    private static final boolean TURRET_SENSOR_PHASE = true;
    private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double VISION_HOMING_BIAS = 0 /* 1.75 */; // deg

    public static final double CARDINAL_SOUTH = 32.556; // deg
    public static final double CARDINAL_WEST = CARDINAL_SOUTH + 90; // deg
    public static final double CARDINAL_NORTH = CARDINAL_SOUTH + 180; // deg
    public static final double MAX_ANGLE = convertTurretTicksToDegrees(TURRET_POSITION_MAX - TURRET_POSITION_MIN);

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");

        turret.setNeutralMode(NeutralMode.Brake);
        turret.setSensorPhase(TURRET_SENSOR_PHASE);

        SmartDashboard.putNumber("TURRET_POSITION_MIN", TURRET_POSITION_MIN);
        SmartDashboard.putNumber("TURRET_POSITION_MAX", TURRET_POSITION_MAX);
        SmartDashboard.putNumber("atan2 Vision", 0);

        this.kP = factory.getConstant(NAME, "kP");
        this.kI = factory.getConstant(NAME, "kI");
        this.kD = factory.getConstant(NAME, "kD");
        this.kF = factory.getConstant(NAME, "kF");

        int absolutePosition = getTurretPosAbsolute();
        turret.setSelectedSensorPosition(absolutePosition, kPIDLoopIDx, Constants.kCANTimeoutMs);

        // Position Control
        double peakOutput = 0.5;

        turret.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        turret.configNominalOutputForward(0, Constants.kCANTimeoutMs);
        turret.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
        turret.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
        turret.configAllowableClosedloopError(kPIDLoopIDx, ALLOWABLE_ERROR_TICKS, Constants.kCANTimeoutMs);

        // Soft Limits
        turret.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turret.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turret.configForwardSoftLimitThreshold(TURRET_POSITION_MAX, Constants.kCANTimeoutMs); // Forward = MAX
        turret.configReverseSoftLimitThreshold(TURRET_POSITION_MIN, Constants.kCANTimeoutMs); // Reverse = MIN
        turret.overrideLimitSwitchesEnable(true);
        turret.overrideSoftLimitsEnable(true);

        // Network Table Listener
        networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        networkTable.addEntryListener("center_x", (table, key, entry, value, flags) -> {
            if (value.getDouble() < 0) { return; }
            var deltaXPixels = (value.getDouble() - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
            this.deltaXAngle = deltaXPixels * (CAMERA_FOV / VIDEO_WIDTH) + VISION_HOMING_BIAS; // Multiply by FOV to pixel ratio
            // TODO: test this formula
            SmartDashboard.getEntry("atan2 vision").setDouble(
                Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH) + VISION_HOMING_BIAS));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void setAutoHomeEnabled(boolean autoHomeEnabled) {
        this.autoHomeEnabled = autoHomeEnabled;
    }

    private void autoHome() {
        setTurretPosition(getTurretPositionTicks() + convertTurretDegreesToTicks(deltaXAngle));
    }

    @Override
    public double getKP() {
        return kP;
    }

    @Override
    public double getKI() {
        return kI;
    }

    @Override
    public double getKD() {
        return kD;
    }

    @Override
    public double getKF() {
        return kF;
    }

    public void setTurretSpeed(double speed) {
        turretSpeed = speed;
        isPercentOutput = true;
        outputsChanged = true;
    }

    public void setTurretPosition(double position) {
        turretPos = position;
        isPercentOutput = false;
        outputsChanged = true;
    }

    public void setTurretAngle(double angle) {
        setTurretPosition(convertTurretDegreesToTicks(angle) + TURRET_POSITION_MIN);
    }

    public double getDeltaX() {
        return deltaXAngle;
    }

    public void jogLeft() {
        setTurretPosition(getTurretPositionTicks() - TURRET_JOG_TICKS);
    }

    public void jogRight() {
        setTurretPosition(getTurretPositionTicks() + TURRET_JOG_TICKS);
    }

    public double getTurretPositionDegrees() {
        return convertTurretTicksToDegrees(getTurretPositionTicks() - TURRET_POSITION_MIN);
    }

    public int getTurretPosAbsolute() {
        if (turret instanceof TalonSRX) {
            int rawValue = ((TalonSRX) turret).getSensorCollection().getPulseWidthPosition() & 0xFFF;
            return (TURRET_SENSOR_PHASE ? -1 : 1) * rawValue;
        }
        return 0;
    }

    public int getTurretPositionTicks() {
        return turret.getSelectedSensorPosition(kPIDLoopIDx);
    }

    public double getTargetPosition() {
        return turretPos;
    }

    public double getPositionError() {
        return turret.getClosedLoopError(kPIDLoopIDx);
    }

    public double getTurretSpeed() {
        return turretSpeed;
    }

    public static double convertTurretDegreesToTicks(double degrees) {
        return (degrees / 360.0) * TURRET_ENCODER_PPR;
    }

    public static double convertTurretTicksToDegrees(int ticks) {
        return (ticks / TURRET_ENCODER_PPR) * 360;
    }

    @Override
    public void writePeriodicOutputs() {
        if (autoHomeEnabled) {
            autoHome();
        }
        if (outputsChanged) {
            if (isPercentOutput) {
                turret.set(ControlMode.PercentOutput, turretSpeed);
            } else {
                turret.set(ControlMode.Position, turretPos);
            }

            outputsChanged = false;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Turret Degrees", this::getTurretPositionDegrees, null);
        builder.addDoubleProperty("Turret Absolute Ticks", this::getTurretPosAbsolute, null);
        builder.addDoubleProperty("Turret Relative Ticks", this::getTurretPositionTicks, null);
        builder.addDoubleProperty("Turret Error", this::getPositionError, null);
    }
}
