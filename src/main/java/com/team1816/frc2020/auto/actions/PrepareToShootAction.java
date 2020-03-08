package com.team1816.frc2020.auto.actions;

import com.team1816.lib.auto.actions.ParallelAction;

public class PrepareToShootAction extends ParallelAction {
    public PrepareToShootAction(double turretAngle) {
        super(
            new TurretAction(turretAngle),
        //    new AutoAimAction(),
            new RampUpShooterAction()
        );
    }
}
