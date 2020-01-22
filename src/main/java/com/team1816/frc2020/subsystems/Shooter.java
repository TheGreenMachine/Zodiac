package com.team1816.frc2020.subsystems;

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

    private Shooter() {
        super(NAME);
        this.shootMain = Robot.getFactory().getMotor(NAME, "shootMain");
        this.shootSlaveA = Robot.getFactory().getMotor(NAME, "shootSlaveA", shootMain);
        this.shootSlaveB = Robot.getFactory().getMotor(NAME, "shootSlaveB", shootMain);
        this.shootSlaveC = Robot.getFactory().getMotor(NAME, "shootSlaveC", shootMain);
    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
