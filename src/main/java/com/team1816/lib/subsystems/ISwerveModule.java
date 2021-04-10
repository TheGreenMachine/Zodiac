package com.team1816.lib.subsystems;

public interface ISwerveModule {
    String getName();

    double getAzimuthVelocity();
    double getAzimuthPosition();
    double getAzimuthError();

    double getDriveVelocity();
    double getDriveError();
}