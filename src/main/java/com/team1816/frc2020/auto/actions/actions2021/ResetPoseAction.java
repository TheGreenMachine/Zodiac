package com.team1816.frc2020.auto.actions.actions2021;

import com.team1816.frc2020.subsystems.Drive;
import com.team1816.lib.auto.actions.Action;
import com.team254.lib.geometry.Pose2d;

public class ResetPoseAction implements Action {
    private Pose2d newPose;
    Drive drive;

    public ResetPoseAction(Pose2d newPose) {
        this.newPose = newPose;

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
        return false;
    }

    @Override
    public void done() {

    }
}
