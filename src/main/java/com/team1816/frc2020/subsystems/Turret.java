package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.RobotState;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
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

    public enum ControlMode {
        FIELD_FOLLOWING,
        CAMERA_FOLLOWING,
        POSITION,
        MANUAL,
    }
    private long lastWriteTime = System.currentTimeMillis();
    private double lastAngle;
    // Components
    private final IMotorControllerEnhanced turret;
    private final Camera camera = Camera.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final LedManager led = LedManager.getInstance();
    private final DistanceManager distanceManager = DistanceManager.getInstance();

    // State
    private double turretPos;
    private double turretSpeed;
    private boolean outputsChanged;
    private double turretAngleRelativeToField;
    private double followTargetTurretSetAngle;
    private ControlMode controlMode = ControlMode.MANUAL;
    private int zone;

    // Constants
    private static final int kPIDLoopIDx = 0;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private static final double TURRET_ENCODER_PPR = factory.getConstant(
        "turret",
        "encPPR"
    );
    private static final int ALLOWABLE_ERROR_TICKS = 5;
    private static final double TURRET_JOG_DEGREES = 1;
    public static final double TURRET_JOG_SPEED = 0.35;
    private static final double TURRET_JOG_TICKS = convertTurretDegreesToTicks(
        TURRET_JOG_DEGREES
    );
    public static final int TURRET_POSITION_MIN =
        ((int) factory.getConstant("turret", "minPos"));
    public static final int TURRET_POSITION_MAX =
        ((int) factory.getConstant("turret", "maxPos"));
    private static final boolean TURRET_SENSOR_PHASE = true;

    public static final double CARDINAL_SOUTH = 32.556; // deg
    public static final double CARDINAL_WEST = CARDINAL_SOUTH + 90; // deg
    public static final double CARDINAL_NORTH = CARDINAL_SOUTH + 180; // deg
    public static final double MAX_ANGLE = convertTurretTicksToDegrees(
        TURRET_POSITION_MAX - TURRET_POSITION_MIN
    );

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
            turret.configAllowableClosedloopError(
                kPIDLoopIDx,
                ALLOWABLE_ERROR_TICKS,
                Constants.kCANTimeoutMs
            );

            // Soft Limits
            turret.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
            turret.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
            turret.configForwardSoftLimitThreshold(
                TURRET_POSITION_MAX,
                Constants.kCANTimeoutMs
            ); // Forward = MAX
            turret.configReverseSoftLimitThreshold(
                TURRET_POSITION_MIN,
                Constants.kCANTimeoutMs
            ); // Reverse = MIN
            turret.overrideLimitSwitchesEnable(true);
            turret.overrideSoftLimitsEnable(true);

            turretAngleRelativeToField = robotState.getLatestFieldToTurret();
        }
    }

    @Override
    public synchronized void zeroSensors() {
        int absolutePosition = getTurretPosAbsolute();
        turret.setSelectedSensorPosition(
            absolutePosition,
            kPIDLoopIDx,
            Constants.kLongCANTimeoutMs
        );
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

    public void setZone(int zone){
        this.zone=zone;
    }

    public void setTurretSpeed(double speed) {
        setControlMode(ControlMode.MANUAL);
        turretSpeed = speed;
        outputsChanged = true;
    }

    public synchronized void setTurretPosition(double position) {
        turretPos = position;
        outputsChanged = true;
    }

    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        setTurretAngleInternal(angle);
    }

    private synchronized void setTurretAngleInternal(double angle) {
        if (angle < MAX_ANGLE - 360) {
            setTurretPosition(
                convertTurretDegreesToTicks(angle + 360) + TURRET_POSITION_MIN
            );
        } else if (angle > 360) {
            setTurretPosition(
                convertTurretDegreesToTicks(angle - 360) + TURRET_POSITION_MIN
            );
        } else if (angle >= 0 && angle <= MAX_ANGLE) {
            setTurretPosition(convertTurretDegreesToTicks(angle) + TURRET_POSITION_MIN);
        }
        // do nothing if angle in deadzone
    }

    public synchronized void lockTurret() {
        setTurretAngle(getTurretPositionDegrees());
    }

    public void jogLeft() {
        setTurretPosition(getTurretPositionTicks() - TURRET_JOG_TICKS);
    }

    public void jogRight() {
        setTurretPosition(getTurretPositionTicks() + TURRET_JOG_TICKS);
    }

    public double getTurretPositionDegrees() {
        return convertTurretTicksToDegrees(
            getTurretPositionTicks() - TURRET_POSITION_MIN
        );
    }

    public int getTurretPosAbsolute() {
        if (turret instanceof TalonSRX) {
            int rawValue =
                ((TalonSRX) turret).getSensorCollection().getPulseWidthPosition() & 0xFFF;
            return (TURRET_SENSOR_PHASE ? -1 : 1) * rawValue;
        }
        return 0;
    }

    public int getTurretPositionTicks() {
        return (int)turret.getSelectedSensorPosition(kPIDLoopIDx);
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

    public double getTurretAngleRelativeToField() {
        return turretAngleRelativeToField;
    }

    public double getFollowTargetTurretSetAngle() {
        return followTargetTurretSetAngle;
    }

    @Override
    public void readPeriodicInputs() {
        turretAngleRelativeToField = robotState.getLatestFieldToTurret();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (controlMode) {
            case CAMERA_FOLLOWING:
                autoHome();
                positionControl();
                break;
//            case FIELD_FOLLOWING:
//                trackGyro();
//                positionControl();
//                break;
            case POSITION:
                positionControl();
                break;
            case MANUAL:
                manualControl();
                break;
        }
        //System.out.println("Turret Bias: "+distanceManager.getTurretBias(zone));
    }

    private void autoHome() {
        if(System.currentTimeMillis()>lastWriteTime+60){
            lastWriteTime=System.currentTimeMillis();
            setTurretAngleInternal(
                getTurretPositionDegrees() +
                    camera.getDeltaXAngle() +
                    distanceManager.getTurretBias(zone)

            );
            lastAngle=getTurretPositionDegrees() +
                camera.getDeltaXAngle() +
                distanceManager.getTurretBias(zone);

        }
        else{
            setTurretAngleInternal(lastAngle);
        }

    }

    private void trackGyro() {
        followTargetTurretSetAngle =
            (getTurretPositionDegrees() - turretAngleRelativeToField);
        setTurretAngleInternal(followTargetTurretSetAngle);
    }

    private void positionControl() {
        if (outputsChanged) {
            turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, turretPos);
            outputsChanged = false;
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            if (turretSpeed == 0) {
                turret.set(
                    com.ctre.phoenix.motorcontrol.ControlMode.Position,
                    getTurretPositionTicks() + 200 * turret.getMotorOutputPercent()
                );
            } else {
                turret.set(
                    com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                    turretSpeed
                );
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
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Turret Degrees", this::getTurretPositionDegrees, null);
        builder.addDoubleProperty(
            "Turret Absolute Ticks",
            this::getTurretPosAbsolute,
            null
        );
        builder.addDoubleProperty(
            "Turret Relative Ticks",
            this::getTurretPositionTicks,
            null
        );
        builder.addDoubleProperty("Turret Error", this::getPositionError, null);
        builder.addDoubleProperty(
            "Angle Set According to Gyro",
            this::getFollowTargetTurretSetAngle,
            null
        );
        builder.addStringProperty(
            "Turret Control Mode",
            () -> getControlMode().name(),
            null
        );
    }
}
