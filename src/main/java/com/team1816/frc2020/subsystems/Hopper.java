package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Robot;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

public class Hopper extends Subsystem {
    private static final String NAME = "hopper";
    private static Hopper INSTANCE;

    public static Hopper getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Hopper();
        }

        return INSTANCE;
    }

    // Components
    private final Solenoid hopperSolenoid;
    private final IMotorControllerEnhanced spindexer;

    // State
    private boolean hopperOut;
    private double spindexerVelocity;
    private boolean outputsChanged;

    private Hopper() {
        super(NAME);
        this.hopperSolenoid = Robot.getFactory().getSolenoid(NAME, "hopper");
        this.spindexer = Robot.getFactory().getMotor(NAME, "solenoid");
    }

    public void setHopperPivot(boolean hopperOut) {
        this.hopperOut = hopperOut;
        outputsChanged = true;
    }

    public void setSpindexer(double spindexerOutput) {
        this.spindexerVelocity = spindexerOutput;
        outputsChanged = true;
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            this.spindexer.set(ControlMode.PercentOutput, spindexerVelocity);
            this.hopperSolenoid.set(hopperOut);
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