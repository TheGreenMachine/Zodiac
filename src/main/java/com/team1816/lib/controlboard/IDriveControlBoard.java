package com.team1816.lib.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getSlowMode();

    boolean getDrivetrainFlipped();

    boolean getCollectorDown();

    boolean getCollectorUp();

    boolean getFeederToTrenchSpline();

    boolean getTrenchToFeederSpline();

    boolean getBrakeMode();

    int getDriverClimber();
}
