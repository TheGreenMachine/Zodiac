package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;

public class Turret extends Subsystem {
    private static final String NAME = "turret";
    private static Turret INSTANCE;

    private final int kTimeoutMs=30;

    private IMotorControllerEnhanced turret;

    public static final int FORWARD_SENSOR_LIMIT = -1;
    public static final int REVERSE_SENSOR_LIMIT = -1;
    /*
    TODO: change forward and reverse sensor limit to actual values
     */
    private double turretPos;
    private double turretSpeed;

    private boolean outputChanged;
    private boolean isPercentOutput;

    public static Turret getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Turret();
        }

        return INSTANCE;
    }

    public Turret(){
        super(NAME);
        RobotFactory factory= Robot.getFactory();
        this.turret=factory.getMotor(NAME,"turret");
    }

    public void setTurretSpeed(double speed){
        turretSpeed=speed;
        isPercentOutput=true;
        outputChanged=true;
    }

    public void setTurretPos(double position){
        turretPos=position;
        isPercentOutput=false;
        outputChanged=true;

        turret.configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT,kTimeoutMs);
        turret.configReverseSoftLimitThreshold(REVERSE_SENSOR_LIMIT,kTimeoutMs);
        turret.configForwardSoftLimitEnable(true,kTimeoutMs);
        turret.configReverseSoftLimitEnable(true,kTimeoutMs);

    }

    public boolean isPercentOutput(){
        return isPercentOutput;
    }

    public double getTurretPosSetPoint(){
        return turretPos;
    }

    public double getTurretSpeed(){
        return turretSpeed;
    }


    @Override
    public void writePeriodicOutputs() {
        if(outputChanged){
            if(isPercentOutput){
                turret.set(ControlMode.PercentOutput,turretSpeed);
            }
            else{
                turret.set(ControlMode.Position,turretPos);
            }
            outputChanged=false;
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
