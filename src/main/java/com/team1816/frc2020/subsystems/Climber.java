package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

import java.util.concurrent.CompletableFuture;

public class Climber extends Subsystem {

    private static final String NAME = "climber";
    private static Climber INSTANCE;

    public static Climber getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Climber();
        }

        return INSTANCE;
    }

    // Components
    private IMotorControllerEnhanced elevator;
    private final ISolenoid deployer;

    // State
    private double climberPow;
    private boolean isDeployed;
    private boolean outputsChanged = false;

    public Climber() {
        super(NAME);
        deployer = factory.getSolenoid(NAME, "deployer");
    }

    @Override
    public CompletableFuture<Void> initAsync() {
        return factory.getMotor(NAME, "elevator")
            .thenAccept(motor -> this.elevator = motor);
    }

    public void setClimberPower(double power) {
        climberPow = power;
        outputsChanged = true;
    }

    public void setDeployed(boolean deployed) {
        this.isDeployed = deployed;
        outputsChanged = true;
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            elevator.set(ControlMode.PercentOutput, climberPow);
            deployer.set(isDeployed);
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }
}
