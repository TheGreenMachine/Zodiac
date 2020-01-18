package com.team1816.frc2019.subsystems;

import com.team1816.lib.subsystems.Subsystem;

public class Shooter extends Subsystem {
    private static final String NAME = "shooter";

    public Shooter(){
        super(NAME);
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
