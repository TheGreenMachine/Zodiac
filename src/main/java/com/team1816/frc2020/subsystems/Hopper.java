package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

public class Hopper extends Subsystem {
    private static final String NAME = "hopper";
    private static Hopper INSTANCE;

    public static Hopper getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Hopper();
        }

        return INSTANCE;
    }

    // Components
    private final Solenoid feederFlap;
    private final IMotorControllerEnhanced spindexer;
    private final IMotorControllerEnhanced elevator;

    // State
    private boolean feederFlapOut;
    private double spindexerPower;
    private double elevatorPower;
    private boolean outputsChanged;

    private boolean lockToShooter;
    private int waitForShooterLoopCounter;
    private boolean shooterWasAtTarget;

    private Hopper() {
        super(NAME);

        this.feederFlap = factory.getSolenoid(NAME, "feederFlap");
        this.spindexer = factory.getMotor(NAME, "spindexer");
        this.elevator = factory.getMotor(NAME, "elevator");
    }

    public void setFeederFlap(boolean feederFlapOut) {
        this.feederFlapOut = feederFlapOut;
        outputsChanged = true;
    }

    public void setSpindexer(double spindexerOutput) {
        this.spindexerPower = 0.25 * spindexerOutput;
        outputsChanged = true;
    }

    public void setElevator(double elevatorOutput) {
        this.elevatorPower = elevatorOutput;
        outputsChanged = true;
    }

    public void setIntake(double intakeOutput) {
        setElevator(intakeOutput);
        setSpindexer(intakeOutput);
    }

    public void lockToShooter(boolean lock) {
        this.lockToShooter = lock;
        this.waitForShooterLoopCounter = 0;
    }

    @Override
    public void writePeriodicOutputs() {
        if (lockToShooter) {
            if (waitForShooterLoopCounter < 10) {
                waitForShooterLoopCounter++;
                return;
            }

            if (!Shooter.getInstance().isVelocityNearTarget()) {
                // Shooter has not sped up yet, wait.
                // if (shooterWasAtTarget) {
                //     this.spindexer.set(ControlMode.PercentOutput, 0);
                //     this.elevator.set(ControlMode.PercentOutput, 0);
                //     shooterWasAtTarget = false;
                // }
                return;
            }
            lockToShooter = false;
            shooterWasAtTarget = true;
        }
        if (outputsChanged) {
            this.spindexer.set(ControlMode.PercentOutput, spindexerPower);
            this.elevator.set(ControlMode.PercentOutput, elevatorPower);
            this.feederFlap.set(feederFlapOut);
            outputsChanged = false;
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
