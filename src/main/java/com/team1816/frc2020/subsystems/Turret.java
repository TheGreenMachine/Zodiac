package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

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

    // State
    private double turretPos;
    private double turretSpeed;
    private boolean outputsChanged;
    private boolean isPercentOutput;

    // Constants
    private static final int kPIDLoopIDx = 0;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private static final double TURRET_ENCODER_PPR = Robot.getFactory().getConstant("turret", "encPPR");
    private static final double TURRET_JOG_DEGREES = 10;
    private static final double TURRET_JOG_TICKS = convertTurretDegreesToTicks(TURRET_JOG_DEGREES);

    public Turret() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.turret = factory.getMotor(NAME, "turret");

        this.kP = factory.getConstant(NAME, "kP");
        this.kI = factory.getConstant(NAME, "kI");
        this.kD = factory.getConstant(NAME, "kD");
        this.kF = factory.getConstant(NAME, "kF");

        int absolutePosition = getTurretPosAbsolute();
        turret.setSelectedSensorPosition(absolutePosition, kPIDLoopIDx, Constants.kCANTimeoutMs);

        double peakOutput = 0.5;

        turret.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        turret.configNominalOutputForward(0, Constants.kCANTimeoutMs);
        turret.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
        turret.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
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
        turretPos = convertTurretDegreesToTicks(position);
        isPercentOutput = false;
        outputsChanged = true;
    }

    public void jogLeft() {
        setTurretPosition(getTurretPositionTicks() - TURRET_JOG_TICKS);
    }

    public void jogRight() {
        setTurretPosition(getTurretPositionTicks() + TURRET_JOG_TICKS);
    }

    public double getTurretPositionDegrees() {
        return convertTurretTicksToDegrees(getTurretPositionTicks());
    }

    public int getTurretPosAbsolute() {
        if (turret instanceof TalonSRX) {
            return ((TalonSRX) turret).getSensorCollection().getPulseWidthPosition() & 0xFFF;
        }
        return 0;
    }

    public int getTurretPositionTicks() {
        return turret.getSelectedSensorPosition(kPIDLoopIDx);
    }

    public double getTurretSpeed() {
        return turretSpeed;
    }

    public static double convertTurretDegreesToTicks(double degrees) {
        return (degrees / 360) * TURRET_ENCODER_PPR;
    }

    public static double convertTurretTicksToDegrees(int ticks) {
        return (ticks / TURRET_ENCODER_PPR) * 360;
    }

    @Override
    public void writePeriodicOutputs() {
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
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Turret Degrees", this::getTurretPositionDegrees, null);
        builder.addDoubleProperty("Turret Absolute Ticks", this::getTurretPosAbsolute, null);
        builder.addDoubleProperty("Turret Relative Ticks", this::getTurretPositionTicks, null);
    }
}
