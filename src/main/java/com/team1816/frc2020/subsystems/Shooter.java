package com.team1816.frc2020.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Robot;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

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

    private final Solenoid hood;

    // States
    private double shootererVelocity;
    private boolean hoodDown;
    private boolean outputsChanged;

    // Constants
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private Shooter() {
        super(NAME);
        this.shooterMain = Robot.getFactory().getMotor(NAME, "shooterMain");
        this.shooterFollowerA = Robot.getFactory().getMotor(NAME, "shooterFollowerA", shooterMain);
        this.shooterFollowerB = Robot.getFactory().getMotor(NAME, "shooterFollowerB", shooterMain);
        this.shooterFollowerC = Robot.getFactory().getMotor(NAME, "shooterFollowerC", shooterMain);
        this.hood = Robot.getFactory().getSolenoid(NAME, "hood");

        this.kP = Robot.getFactory().getConstant("kP");
        this.kI = Robot.getFactory().getConstant("kP");
        this.kD = Robot.getFactory().getConstant("kP");
        this.kF = Robot.getFactory().getConstant("kP");
    }

    public void setVelocity(double velocity) {
        this.shootererVelocity = velocity;
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
        return shootererVelocity;
    }

    public double getError() {
        return shooterMain.getClosedLoopError(0);
    }

    public void initLogger() {
        BadLog.createTopic("Shooter/ActVel", "Native Units", INSTANCE::getActualVelocity, "hide",
            "join:Shooter/Velocities");
        BadLog.createTopic("Shooter/TargetVel", "Native Units", INSTANCE::getTargetVelocity, "hide",
            "join:Shooter/Velocities");
        BadLog.createTopic("Shooter/Error", "Native Units", INSTANCE::getError, "hide",
            "join:Shooter/Velocities");
    }
    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            this.shooterMain.set(ControlMode.Velocity, shootererVelocity);
            this.hood.set(hoodDown);
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
