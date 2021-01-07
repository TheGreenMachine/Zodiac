package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

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
    private final IMotorControllerEnhanced elevator;
    private final ISolenoid deployer;

    // State
    private double climberPow;
    private boolean isDeployed;
    private boolean outputsChanged = false;

    public Climber() {
        super(NAME);
        elevator = factory.getMotor(NAME, "elevator");
        deployer = factory.getSolenoid(NAME, "deployer");
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
