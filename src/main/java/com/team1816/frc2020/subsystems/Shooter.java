package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1816.frc2020.Constants;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem implements PidProvider {

    public static Shooter INSTANCE;
    public static final String NAME = "shooter";

    public static Shooter getInstance(){
        if (INSTANCE==null){
            return new Shooter(NAME);
        }
        else{
            return INSTANCE;
        }
    }

    private Shooter(String name) {
        super(name);
    }


    @Override
    public double getKP() {
        return 0;
    }

    @Override
    public double getKI() {
        return 0;
    }

    @Override
    public double getKD() {
        return 0;
    }

    @Override
    public double getKF() {
        return 0;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}
