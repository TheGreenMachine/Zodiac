package com.team1816.lib.subsystems;

public interface SwerveDrivetrain {
    ISwerveModule[] getSwerveModules();

    double getHeadingDegrees();
    double getDesiredHeading();
}
