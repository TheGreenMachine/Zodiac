package com.team1816.frc2019.subsystems;

import com.team1816.lib.subsystems.Subsystem;

public class Turret extends Subsystem {
    private static final String NAME = "turret";

    public Turret(){
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
