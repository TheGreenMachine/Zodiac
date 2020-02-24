package com.team1816.lib.controlboard;

public interface IButtonControlBoard {
    void reset();

    void setRumble(boolean on);

    boolean getSpinnerReset();

    boolean getFeederFlapOut();

    boolean getFeederFlapIn();

    double getClimber();

    boolean getSpinnerColor();

    boolean getSpinnerThreeTimes();

    boolean getTurretJogLeft();

    boolean getTurretJogRight();

    boolean getShoot();

    boolean getAutoHome();

    boolean getCollectorBackSpin();



}
