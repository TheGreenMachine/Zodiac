package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Robot;
import com.team1816.lib.hardware.RobotFactory;
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
    private final IMotorControllerEnhanced climberMotor;

    // State
    private double climberPow;
    private boolean outputsChanged = false;

    public Climber() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();
        climberMotor = factory.getMotor(NAME, "elevator");
    }

    public void setClimberPower(double power) {
        climberPow = power;
        outputsChanged = true;
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            climberMotor.set(ControlMode.PercentOutput, climberPow);
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
