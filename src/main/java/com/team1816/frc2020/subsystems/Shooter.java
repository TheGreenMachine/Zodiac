package com.team1816.frc2020.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Robot;
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
    private final IMotorControllerEnhanced shootMain;
    private final IMotorController shootSlaveA;
    private final IMotorController shootSlaveB;
    private final IMotorController shootSlaveC;

    // States
    private double shooterVelocity;
    private boolean outputsChanged;

    private Shooter() {
        super(NAME);
        this.shootMain = Robot.getFactory().getMotor(NAME, "shootMain");
        this.shootSlaveA = Robot.getFactory().getMotor(NAME, "shootSlaveA", shootMain);
        this.shootSlaveB = Robot.getFactory().getMotor(NAME, "shootSlaveB", shootMain);
        this.shootSlaveC = Robot.getFactory().getMotor(NAME, "shootSlaveC", shootMain);
    }

    public void setVelocity(double velocity) {
        this.shooterVelocity = velocity;
        outputsChanged = true;
    }

    public double getActualVelocity() {
        return shootMain.getSelectedSensorVelocity(0);
    }

    public double getTargetVelocity() {
        return shooterVelocity;
    }

    public double getError() {
        return shootMain.getClosedLoopError(0);
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
            this.shootMain.set(ControlMode.Velocity, shooterVelocity);
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
