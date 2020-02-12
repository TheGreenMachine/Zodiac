package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;


public class Collector extends Subsystem {
    private static final String NAME = "collector";
    private static Collector INSTANCE;

    // Components
    private final Solenoid armPiston;
    private final IMotorControllerEnhanced intake;

    // State
    private double intakePow;
    private boolean armDown;
    private boolean outputsChanged = false;


    public static Collector getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Collector();
        }

        return INSTANCE;
    }

    private Collector() {
        super(NAME);
        RobotFactory factory = Robot.getFactory();

        this.armPiston = factory.getSolenoid(NAME, "arm");
        this.intake = factory.getMotor(NAME, "intake");
    }

    public boolean isArmDown() {
        return this.armDown;
    }

    public double getIntakePow() {
        return this.intakePow;
    }

    public void setArm(boolean down) {
        this.armDown = down;
        this.outputsChanged = true;
    }

    public void setIntakePow(double intakePower) {
        this.intakePow = intakePower;
        outputsChanged = true;
    }

    public void setDeployed(boolean down) {
        if (down) {
          //  setArm(true);
            setIntakePow(0.5);
        } else {
            setIntakePow(0);
         //   setArm(false);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
     //       this.armPiston.set(armDown);
            this.intake.set(ControlMode.PercentOutput, intakePow);
            this.outputsChanged = false;
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
