package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.lib.auto.actions.ParallelAction;

public class PrepareToShootAction extends ParallelAction {

    public PrepareToShootAction(double turretAngle) {
        super(
            new TurretAction(turretAngle),
            new AutoAimAction(),
            new RampUpShooterAction(),
            new LoadBallsAction()
        );
    }
}
