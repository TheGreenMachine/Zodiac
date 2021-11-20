package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.lib.auto.actions.Action;

public class RampUpShooterAction implements Action {

    @Override
    public void start() {
        Shooter.getInstance().startShooter();
        Shooter.getInstance().autoHood();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        System.out.println("Has met Autoaim conditions");
        return true;
    }

    @Override
    public void done() {}
}
