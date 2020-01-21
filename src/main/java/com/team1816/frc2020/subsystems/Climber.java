package com.team1816.frc2020.subsystems;

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

    public Climber(){
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
