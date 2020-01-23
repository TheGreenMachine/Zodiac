package com.team1816.frc2020.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1816.frc2020.Constants;
import com.team1816.frc2020.Robot;
import com.team1816.lib.subsystems.Subsystem;

public class Shooter extends Subsystem {
    private static final String NAME = "shooter";
    private static Shooter INSTANCE;

    public static Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Shooter();
        }

        return INSTANCE;
    }

    // Components
    private final IMotorControllerEnhanced shooterMain;
    private final IMotorController shooterFollowerA;
    private final IMotorController shooterFollowerB;
    private final IMotorController shooterFollowerC;

//    private final Solenoid hood;

    // State
    private double shooterVelocity;
    private boolean hoodDown;
    private boolean outputsChanged;

    // Constants
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private Shooter() {
        super(NAME);
        this.shooterMain = Robot.getFactory().getMotor(NAME, "shooterMain");
        this.shooterFollowerA = Robot.getFactory().getMotor(NAME, "shooterFollowerA", shooterMain);
        this.shooterFollowerB = Robot.getFactory().getMotor(NAME, "shooterFollowerB", shooterMain);
        this.shooterFollowerC = Robot.getFactory().getMotor(NAME, "shooterFollowerC", shooterMain);
     //   this.hood = Robot.getFactory().getSolenoid(NAME, "hood");

        this.kP = Robot.getFactory().getConstant(NAME, "kP");
        this.kI = Robot.getFactory().getConstant(NAME, "kI");
        this.kD = Robot.getFactory().getConstant(NAME, "kD");
        this.kF = Robot.getFactory().getConstant(NAME, "kF");

        shooterMain.setNeutralMode(NeutralMode.Coast);
        shooterFollowerA.setNeutralMode(NeutralMode.Coast);
        shooterFollowerB.setNeutralMode(NeutralMode.Coast);
        shooterFollowerC.setNeutralMode(NeutralMode.Coast);

        shooterMain.configClosedloopRamp(2, Constants.kCANTimeoutMs);
    }

    public void setVelocity(double velocity) {
        this.shooterVelocity = velocity;
        outputsChanged = true;
    }

    public void setHoodDown(boolean hoodDown) {
        this.hoodDown = hoodDown;
        outputsChanged = true;
    }

   public void initLogger() {
      BadLog.createTopic("Shooter/ActVel", "Native Units", this::getActualVelocity,
           "hide", "join:Shooter/Velocities");
       BadLog.createTopic("Shooter/TargetVel", "Native Units", this::getTargetVelocity,
           "hide", "join:Shooter/Velocities");
       BadLog.createTopic("Shooter/Error", "Native Units", this::getError,
           "hide", "join:Shooter/Velocities");
   }

    public double getActualVelocity() {
        return shooterMain.getSelectedSensorVelocity(0);
    }

    public double getTargetVelocity() {
        return shooterVelocity;
    }

    public double getError() {
        return shooterMain.getClosedLoopError(0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            System.out.println("Shooter velocity: " + shooterVelocity);
            this.shooterMain.set(ControlMode.Velocity, shooterVelocity);
        //    this.hood.set(hoodDown);
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
