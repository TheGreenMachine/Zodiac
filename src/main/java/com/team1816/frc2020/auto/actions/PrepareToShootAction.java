package com.team1816.frc2020.auto.actions;

import com.team1816.lib.auto.actions.ParallelAction;

public class PrepareToShootAction extends ParallelAction {

    public PrepareToShootAction(double turretAngle, int zone, boolean preLoad){
        super(
            new TurretAction(turretAngle),
            new AutoAimAction(zone),
            new RampUpShooterAction(zone),
            new LoadBallsAction(preLoad)


        );
        System.out.println("higher zone is: "+zone);
    }

    public PrepareToShootAction(double turretAngle, int zone){
        super(
            new TurretAction(turretAngle),
            new AutoAimAction(zone),
            new RampUpShooterAction(zone)


        );
        System.out.println("higher zone is: "+zone);
    }

    public PrepareToShootAction(double turretAngle) {
        super(
            new TurretAction(turretAngle),
            new AutoAimAction(),
            new RampUpShooterAction()
        );
    }
}
