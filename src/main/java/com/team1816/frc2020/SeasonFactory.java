package com.team1816.frc2020;

import com.google.inject.Singleton;
import com.team1816.frc2020.subsystems.Drive;
import com.team1816.frc2020.subsystems.SwerveDrive;
import com.team1816.frc2020.subsystems.WestCoastDrive;
import com.team1816.lib.hardware.RobotFactory;

@Singleton
public class SeasonFactory implements Drive.Factory {

    private final RobotFactory factory = Robot.getFactory();
    private static Drive mDrive;

    @Override
    public Drive getInstance() {
        if (mDrive == null) {
            boolean isSwerve = factory.getConstant(Drive.NAME, "isSwerve") == 1;
            if (isSwerve) {
                mDrive = new SwerveDrive();
            } else {
                mDrive = new WestCoastDrive();
            }
        }
        return mDrive;
    }
}
