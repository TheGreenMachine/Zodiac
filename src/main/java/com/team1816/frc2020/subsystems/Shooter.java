package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;

public class Shooter extends Subsystem {
    private static final String NAME = "shooter";
    private static Shooter INSTANCE;

    public static Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Shooter();
        }

        return INSTANCE;
    }

    // Components
    private final IMotorControllerEnhanced shooterMain;
    private final IMotorController shooterFollowerA;
    private final IMotorController shooterFollowerB;
    private final IMotorController shooterFollowerC;

//    private final Solenoid hood;

    // State
    private double shooterVelocity;
    private boolean hoodDown;
    private boolean outputsChanged;

    // Constants
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private Shooter() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.shooterMain = factory.getMotor(NAME, "shooterMaster");
        this.shooterFollowerA = factory.getMotor(NAME, "shooterFollowerA", shooterMain);
        this.shooterFollowerB = factory.getMotor(NAME, "shooterFollowerB", shooterMain);
        this.shooterFollowerC = factory.getMotor(NAME, "shooterFollowerC", shooterMain);
     //   this.hood = factory.getSolenoid(NAME, "hood");

        this.kP = factory.getConstant(NAME, "kP");
        this.kI = factory.getConstant(NAME, "kI");
        this.kD = factory.getConstant(NAME, "kD");
        this.kF = factory.getConstant(NAME, "kF");

        shooterMain.setNeutralMode(NeutralMode.Coast);
        shooterFollowerA.setNeutralMode(NeutralMode.Coast);
        shooterFollowerB.setNeutralMode(NeutralMode.Coast);
        shooterFollowerC.setNeutralMode(NeutralMode.Coast);

        shooterMain.configClosedloopRamp(1, Constants.kCANTimeoutMs);
        shooterMain.setSensorPhase(true);

        setShooterGains();
    }

    private void setShooterGains() {
        shooterMain.config_kP(0, kP, Constants.kLongCANTimeoutMs);
        shooterMain.config_kI(0, kI, Constants.kLongCANTimeoutMs);
        shooterMain.config_kD(0, kD, Constants.kLongCANTimeoutMs);
        shooterMain.config_kF(0, kF, Constants.kLongCANTimeoutMs);
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }

    public double getKF() {
        return kF;
    }

    public void setVelocity(double velocity) {
        this.shooterVelocity = velocity;
        outputsChanged = true;
    }

    public void setHoodDown(boolean hoodDown) {
        this.hoodDown = hoodDown;
        outputsChanged = true;
    }

    public double getActualVelocity() {
        return shooterMain.getSelectedSensorVelocity(0);
    }

    public double getTargetVelocity() {
        return shooterVelocity;
    }

    public double getError() {
        return shooterMain.getClosedLoopError(0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            System.out.println("Shooter velocity: " + shooterVelocity);
            this.shooterMain.set(ControlMode.Velocity, shooterVelocity);
        //    this.hood.set(hoodDown);
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
