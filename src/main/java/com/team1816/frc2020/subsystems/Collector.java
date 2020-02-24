package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;


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

    private boolean isRaising;
    private double startTime;


    public static Collector getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Collector();
        }

        return INSTANCE;
    }

    private Collector() {
        super(NAME);

        this.armPiston = factory.getSolenoid(NAME, "arm");
        this.intake = factory.getMotor(NAME, "intake");

        ((TalonSRX) intake).enableCurrentLimit(true);
        ((TalonSRX) intake).configContinuousCurrentLimit(20);
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
            setArm(true);
            setIntakePow(1);
        } else {
            isRaising = true;
            startTime = Timer.getFPGATimestamp();
            setArm(false);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (isRaising) {
            if ((Timer.getFPGATimestamp() - startTime) > 2) {
                System.out.println("Raising timer passed at : " + (Timer.getFPGATimestamp() - startTime));
                setIntakePow(0);
                isRaising = false;
            }
        }

        if (outputsChanged) {
            this.armPiston.set(armDown);
            this.intake.set(ControlMode.PercentOutput, intakePow);
            this.outputsChanged = false;
        }
    }
    public void setCollectorBackSpin(boolean spin) {
        if (spin) {
            intakePow = 0.2;
            outputsChanged = true;
        } else {
            intakePow = 0.0;
            outputsChanged = true;
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
