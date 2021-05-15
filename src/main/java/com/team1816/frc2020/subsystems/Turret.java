package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.RobotState;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Turret extends Subsystem implements PidProvider {
    private static final String NAME = "turret";
    private static Turret INSTANCE;

    public static Turret getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Turret();
        }
        return INSTANCE;
    }

    public enum ControlMode {
        FIELD_FOLLOWING,
        CAMERA_FOLLOWING,
        POSITION,
        MANUAL
    }

    // Components
    private final IMotorControllerEnhanced turret;
    private final Camera camera = Camera.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final LedManager led = LedManager.getInstance();
    private final DistanceManager distanceManager = DistanceManager.getInstance();

    // State
    private int desiredTurretPos = 0;
    private int followingTurretPos = 0;
    private double turretSpeed;
    private boolean outputsChanged;
    private double turretAngleRelativeToField;
    private ControlMode controlMode = ControlMode.MANUAL;

    // Constants
    private static final int kPIDLoopIDx = 0;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private static final int TURRET_ENCODER_PPR = (int) factory.getConstant(NAME, "encPPR");
    private static final int TURRET_ENCODER_MASK = TURRET_ENCODER_PPR - 1;
    private static final int ALLOWABLE_ERROR_TICKS = 5;
    private static final double TURRET_JOG_DEGREES = 1;
    public static final double TURRET_JOG_SPEED = 0.35;
    private static final double TURRET_JOG_TICKS = convertTurretDegreesToTicks(TURRET_JOG_DEGREES);
    public static final int TURRET_POSITION_MIN = ((int) factory.getConstant(NAME, "minPos"));
    public static final int TURRET_POSITION_MAX = ((int) factory.getConstant(NAME, "maxPos"));
    public static final int ABS_TICKS_SOUTH = ((int) factory.getConstant(NAME, "absPosTicksSouth"));
    private static final boolean TURRET_SENSOR_PHASE = factory.getConstant(NAME, "invertSensorPhase") >= 1;

    public static final double CARDINAL_SOUTH = 0; // deg
    public static final double CARDINAL_WEST = 90; // deg
    public static final double CARDINAL_NORTH = 180; // deg
    public static final double CARDINAL_EAST = 270; // deg
    public static final double MAX_ANGLE = convertTurretTicksToDegrees(TURRET_POSITION_MAX - TURRET_POSITION_MIN);

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");

        turret.setNeutralMode(NeutralMode.Brake);
        turret.setSensorPhase(TURRET_SENSOR_PHASE);

        SmartDashboard.putNumber("TURRET_POSITION_MIN", TURRET_POSITION_MIN);
        SmartDashboard.putNumber("TURRET_POSITION_MAX", TURRET_POSITION_MAX);

        this.kP = factory.getConstant(NAME, "kP");
        this.kI = factory.getConstant(NAME, "kI");
        this.kD = factory.getConstant(NAME, "kD");
        this.kF = factory.getConstant(NAME, "kF");

        synchronized (this) {
            this.zeroSensors();

            // Position Control
            double peakOutput = 0.75;

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

            turretAngleRelativeToField = robotState.getHeadingRelativeToInitial().getDegrees();
        }
    }

    @Override
    public synchronized void zeroSensors() {
        if (turret instanceof TalonSRX) {
            var sensors = ((TalonSRX) turret).getSensorCollection();
            sensors.setQuadraturePosition(sensors.getPulseWidthPosition() & TURRET_ENCODER_MASK, Constants.kLongCANTimeoutMs);
        }
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            if (controlMode == ControlMode.CAMERA_FOLLOWING) {
                if (Constants.kUseAutoAim) {
                    this.controlMode = controlMode;
                    camera.setEnabled(true);
                    led.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
                }
            } else {
                this.controlMode = controlMode;
                camera.setEnabled(false);
                if (controlMode == ControlMode.MANUAL) {
                    led.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                } else {
                    led.indicateDefaultStatus();
                }
            }
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
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
        setControlMode(ControlMode.MANUAL);
        if(turretSpeed != speed) {
            turretSpeed = speed;
            outputsChanged = true;
        }
    }

    public synchronized void setTurretPosition(double position) {
        //Since we are using position we need ensure value stays in one rotation
        int adjPos = (int) position & TURRET_ENCODER_MASK;
        if(desiredTurretPos != adjPos) {
            desiredTurretPos = adjPos;
            outputsChanged = true;
        }
    }

    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        setTurretAngleInternal(angle);
    }

    private synchronized void setTurretAngleInternal(double angle) {
        setTurretPosition(convertTurretDegreesToTicks(angle));
        if (angle < MAX_ANGLE - 360) {
            //setTurretPosition(convertTurretDegreesToTicks(angle + 360));
        } else if (angle > 360) {
            //setTurretPosition(convertTurretDegreesToTicks(angle - 360));
        } else if (angle >= 0 && angle <= MAX_ANGLE) {
            //setTurretPosition(convertTurretDegreesToTicks(angle));
        }
        // do nothing if angle in deadzone
    }

    public synchronized void lockTurret() {
        setTurretAngle(getActualTurretPositionDegrees());
    }

    public void jogLeft() {
        setTurretPosition(getActualTurretPositionTicks() - TURRET_JOG_TICKS);
    }

    public void jogRight() {
        setTurretPosition(getActualTurretPositionTicks() + TURRET_JOG_TICKS);
    }

    public double getActualTurretPositionDegrees() {
        return convertTurretTicksToDegrees(getActualTurretPositionTicks());
    }

    public int getTurretPosAbsolute() {
        if (turret instanceof TalonSRX) {
            int rawValue = ((TalonSRX) turret).getSensorCollection().getPulseWidthPosition() & TURRET_ENCODER_MASK;
            return (TURRET_SENSOR_PHASE ? -1 : 1) * rawValue;
        }
        return 0;
    }

    public int getActualTurretPositionTicks() {
        return (int)turret.getSelectedSensorPosition(kPIDLoopIDx);
    }

    public double getTargetPosition() {
        return followingTurretPos;
    }

    public double getPositionError() {
        return turret.getClosedLoopError(kPIDLoopIDx);
    }

    public static int convertTurretDegreesToTicks(double degrees) {
        return (int)(((degrees) / 360.0) * TURRET_ENCODER_PPR) + ABS_TICKS_SOUTH;
    }

    public static double convertTurretTicksToDegrees(double ticks) {
        var adjTicks = (ticks - ABS_TICKS_SOUTH);
        return adjTicks / TURRET_ENCODER_PPR * 360;
    }

    @Override
    public void readPeriodicInputs() {
        turretAngleRelativeToField =  robotState.getHeadingRelativeToInitial().getDegrees();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (controlMode) {
            case CAMERA_FOLLOWING:
                autoHome();
                positionControl();
                break;
            case FIELD_FOLLOWING:
                trackGyro();
                positionControl();
                break;
            case POSITION:
                followingTurretPos = 0;
                positionControl();
                break;
            case MANUAL:
                manualControl();
                break;
        }
    }

    private void autoHome() {
        setTurretAngleInternal(getActualTurretPositionDegrees() +
            camera.getDeltaXAngle() + distanceManager.getTurretBias(camera.getDistance()));
    }

    private void trackGyro() {
        int fieldTickOffset = convertTurretDegreesToTicks(turretAngleRelativeToField) - ABS_TICKS_SOUTH;
        int adj = desiredTurretPos + fieldTickOffset;
        // Valid positions are 0 to encoder max ticks if we go negative adjust
        if(adj < 0) adj += TURRET_ENCODER_PPR;
        if(adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void positionControl() {
        if (outputsChanged) {
            turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, followingTurretPos);
            outputsChanged = false;
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            if (turretSpeed == 0) {
                turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, getActualTurretPositionTicks() + 200 * turret.getMotorOutputPercent());
            } else {
                turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, turretSpeed);
            }
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {
        camera.setEnabled(false);
    }

    @Override
    public boolean checkSystem() {
        return EnhancedMotorChecker.checkMotors(
            this,
            getTalonCheckerConfig(turret),
            new EnhancedMotorChecker.NamedMotor("turret", turret)
        );
    }

    private EnhancedMotorChecker.CheckerConfig getTalonCheckerConfig(IMotorControllerEnhanced talon) {
        return EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, talon);
    }

}
