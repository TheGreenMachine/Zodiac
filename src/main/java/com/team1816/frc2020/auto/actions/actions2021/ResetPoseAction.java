package com.team1816.frc2020.auto.actions.actions2021;

import com.team1816.frc2020.subsystems.Drive;
import com.team1816.lib.auto.actions.Action;

public class ResetPoseAction implements Action {
    Drive drive;

    public ResetPoseAction() {
    }

    @Override
    public void start() {
        drive.zeroSensors();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
