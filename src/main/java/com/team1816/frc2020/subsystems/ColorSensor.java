package com.team1816.frc2020.subsystems;

import com.revrobotics.ColorSensorV3;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.I2C;

public class ColorSensor extends Subsystem {

    private static final String NAME = "colorSensor";
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private boolean isBlue = true;

    public ColorSensor(){
        super(NAME);
    }

    public void setTeam(boolean isBlue){
        this.isBlue = isBlue;
    }

    public boolean isTeamBall(){
        return (colorSensor.getBlue()>colorSensor.getRed()) == isBlue;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}
