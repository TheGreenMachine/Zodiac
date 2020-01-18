package com.team1816.frc2019.subsystems;

import com.team1816.lib.subsystems.Subsystem;

public class Hopper extends Subsystem {
    private static final String NAME = "hopper";

    public Hopper(){
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
