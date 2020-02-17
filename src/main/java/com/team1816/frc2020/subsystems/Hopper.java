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
    private boolean waitForShooter;
    private int waitForShooterLoopCounter;

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

    public void waitForShooter(boolean wait) {
        this.waitForShooter = wait;
        this.waitForShooterLoopCounter = 0;
    }

    @Override
    public void writePeriodicOutputs() {
        if (waitForShooter) {
            if (waitForShooterLoopCounter < 10) {
                waitForShooterLoopCounter++;
                return;
            }

            if (Math.abs(Shooter.getInstance().getError()) > Shooter.VELOCITY_THRESHOLD) {
//              System.out.println("WAITING FOR SHOOTER!");
                return;
            } else {
                waitForShooter = false;
//              System.out.println("Stopped waiting for shooter at " + Timer.getFPGATimestamp());
            }
        }
        if (outputsChanged) {
            if (Math.abs(Shooter.getInstance().getError()) > Shooter.VELOCITY_THRESHOLD) {
                waitForShooter = true;
                outputsChanged = false;
                return;
            }

            this.spindexer.set(ControlMode.PercentOutput, spindexerPower);
            this.elevator.set(ControlMode.PercentOutput, elevatorPower);
            // this.feederFlap.set(feederFlapOut);
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
