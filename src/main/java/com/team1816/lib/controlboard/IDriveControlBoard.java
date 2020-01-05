package com.team1816.lib.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getShoot();

    boolean getWantsLowGear();

    boolean getThrust();

    boolean getSlowMode();

    boolean getDrivetrainFlipped();

    boolean getEjectBeak();

    boolean getReleaseBeak();

    double getCargoIntake();

    boolean getClimberAndCameraPiston();
}
