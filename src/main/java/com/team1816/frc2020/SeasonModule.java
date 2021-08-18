package com.team1816.frc2020;

import com.google.inject.AbstractModule;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.frc2020.controlboard.ControlBoard;
import com.team1816.frc2020.controlboard.GamepadButtonControlBoard;
import com.team1816.frc2020.controlboard.GamepadDriveControlBoard;

public class SeasonModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(IControlBoard.class).to(ControlBoard.class);
        bind(IDriveControlBoard.class).to(GamepadDriveControlBoard.class);
        bind(IButtonControlBoard.class).to(GamepadButtonControlBoard.class);
    }
}
