package com.team1816.frc2020.auto.actions.actions2020;

import com.team1816.frc2020.subsystems.Turret;
import com.team1816.lib.auto.actions.ParallelAction;

import javax.inject.Inject;

public class PrepareToShootAction extends ParallelAction {
    public PrepareToShootAction(double turretAngle, Turret turret) {
        super(
//            new TurretAction(turretAngle),
            new AutoAimAction(turret),
            new RampUpShooterAction()
//            new LoadBallsAction()
        );
    }
}
