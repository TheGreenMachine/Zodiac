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
    private final IMotorControllerEnhanced shootMain;
    private final IMotorController shootFollowerA;
    private final IMotorController shootFollowerB;
    private final IMotorController shootFollowerC;

    private final Solenoid hood;

    // States
    private double shooterVelocity;
    private boolean outputsChanged;


    private Shooter() {
        super(NAME);
        this.shootMain = Robot.getFactory().getMotor(NAME, "shootMain");
        this.shootFollowerA = Robot.getFactory().getMotor(NAME, "shootFollowerA", shootMain);
        this.shootFollowerB = Robot.getFactory().getMotor(NAME, "shootFollowerB", shootMain);
        this.shootFollowerC = Robot.getFactory().getMotor(NAME, "shootFollowerC", shootMain);
        this.hood = Robot.getFactory().getSolenoid(NAME, "hood");
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
