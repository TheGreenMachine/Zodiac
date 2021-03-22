package com.team1816.frc2020.controlboard;

import java.util.Arrays;
import java.util.List;

public class ActionManager {

    private List<ControlUtils.ButtonAction> actions;

    public ActionManager(ControlUtils.ButtonAction... actions) {
        this.actions = Arrays.asList(actions);
    }

    public void update() {
        actions.forEach(ControlUtils.ButtonAction::update);
    }
}
