package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.controlboard.*;
import com.team1816.frc2020.controlboard.ControlUtils;

public class LibModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Controller.Factory.class).to(ControlUtils.class);
        requestStaticInjection(DriveTrajectory.class);
        //None of these four actions are actually used in the project -- remove?
        requestStaticInjection(DriveOpenLoopAction.class);
        requestStaticInjection(DrivePathAction.class);
        requestStaticInjection(WaitForPathMarkerAction.class);
        requestStaticInjection(ForceEndPathAction.class);
    }
}
