package com.team1816.lib.controlboard;

public interface IButtonControlBoard {
    void reset();

    void setRumble(boolean on);

    boolean getSpinnerReset();

    boolean getFeederFlapperOut();

    boolean getFeederFlapperIn();

    double getClimber();

    boolean getSpinnerColor();

    boolean getSpinnerThreeTimes();
}
