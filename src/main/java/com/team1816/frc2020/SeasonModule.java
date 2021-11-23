package com.team1816.frc2020;

import com.google.inject.AbstractModule;
import com.team1816.frc2020.auto.actions.actions2020.*;
import com.team1816.frc2020.controlboard.ControlBoard;
import com.team1816.frc2020.controlboard.GamepadButtonControlBoard;
import com.team1816.frc2020.controlboard.GamepadDriveControlBoard;
import com.team1816.frc2020.subsystems.*;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.subsystems.Infrastructure;

public class SeasonModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Drive.Factory.class).to(SeasonFactory.class);
        bind(IControlBoard.class).to(ControlBoard.class);
        bind(IDriveControlBoard.class).to(GamepadDriveControlBoard.class);
        bind(IButtonControlBoard.class).to(GamepadButtonControlBoard.class);
        requestStaticInjection(SwerveKinematics.class);
        requestStaticInjection(Drive.class);
        requestStaticInjection(Infrastructure.class);
        requestStaticInjection(Shooter.class);
        requestStaticInjection(Hopper.class);
        requestStaticInjection(Camera.class);
        requestStaticInjection(Turret.class);
        requestStaticInjection(DistanceManager.class);
        requestStaticInjection(TurretAction.class);
        requestStaticInjection(AutoAimAction.class);
        requestStaticInjection(ShootAction.class);
        requestStaticInjection(RampUpShooterAction.class);
        requestStaticInjection(CollectAction.class);
        requestStaticInjection(LoadBallsAction.class);
    }
}
