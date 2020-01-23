package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.frc2020.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;

public class Turret extends Subsystem {
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
    private static final int kTimeoutMs = 10;

    private static double TURRET_ENCODER_PPR;

    public Turret() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();
        this.turret = factory.getMotor(NAME, "turret");

        int absolutePosition = getTurretPosAbsolute();
        turret.setSelectedSensorPosition(absolutePosition, kPIDLoopIDx, kTimeoutMs);
    }

    public void setTurretSpeed(double speed) {
        turretSpeed = speed;
        isPercentOutput = true;
        outputsChanged = true;
    }

    public void setTurretPos(double position) {
        turretPos = position;
        isPercentOutput = false;
        outputsChanged = true;
    }

    public boolean isPercentOutput() {
        return isPercentOutput;
    }

    public int getTurretPosAbsolute() {
        if (turret != null) {
            return ((TalonSRX) turret).getSensorCollection().getPulseWidthPosition() & 0xFFF;
        }
        return 0;
    }

    public int getTurretPos() {
        return turret.getSelectedSensorPosition(kPIDLoopIDx);
    }

    public double getTurretSpeed() {
        return turretSpeed;
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
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
