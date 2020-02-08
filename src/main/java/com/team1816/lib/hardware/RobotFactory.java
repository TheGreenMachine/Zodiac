package com.team1816.lib.hardware;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;

import com.team1816.frc2020.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

public class RobotFactory {

    private YamlConfig config;
    private static boolean verbose;

    public RobotFactory(String configName) {
        System.out.println("Loading Config for " + configName);
        config = YamlConfig.loadFrom(this.getClass().getClassLoader().getResourceAsStream(configName + ".config.yml"));
        verbose = getConstant("verbose") >= 1;
    }

    public boolean isImplemented(String subsystem) {
        return (getSubsystem(subsystem) != null) && (getSubsystem(subsystem).implemented);
    }

    public IMotorControllerEnhanced getMotor(String subsystemName, String name) { // TODO: optimize this method
        if (isImplemented(subsystemName)) {
            YamlConfig.SubsystemConfig subsystem = getSubsystem(subsystemName);
            if (isHardwareValid(subsystem.talons.get(name))) {
                var motor = CtreMotorFactory.createDefaultTalon(subsystem.talons.get(name), false);
                if (subsystem.invertMotor.contains(motor.getDeviceID())) {
                    System.out.println("Inverting " + name);
                    motor.setInverted(true);
                }
                motor.config_kP(0, getConstant(subsystemName, "kP", 0), Constants.kLongCANTimeoutMs);
                motor.config_kI(0, getConstant(subsystemName, "kI", 0), Constants.kLongCANTimeoutMs);
                motor.config_kD(0, getConstant(subsystemName, "kD", 0), Constants.kLongCANTimeoutMs);
                motor.config_kF(0, getConstant(subsystemName, "kF", 0), Constants.kLongCANTimeoutMs);
                return motor;
            } else if (isHardwareValid(subsystem.falcons.get(name))) {
                var motor = CtreMotorFactory.createDefaultTalon(subsystem.falcons.get(name), true);
                if (subsystem.invertMotor.contains(motor.getDeviceID())) {
                    System.out.println("Inverting" + name);
                    motor.setInverted(true);
                }
                motor.config_kP(0, getConstant(subsystemName, "kP", 0), Constants.kLongCANTimeoutMs);
                motor.config_kI(0, getConstant(subsystemName, "kI", 0), Constants.kLongCANTimeoutMs);
                motor.config_kD(0, getConstant(subsystemName, "kD", 0), Constants.kLongCANTimeoutMs);
                motor.config_kF(0, getConstant(subsystemName, "kF", 0), Constants.kLongCANTimeoutMs);
                return motor;
            } // Never make the victor a master
        }
        DriverStation.reportWarning("Warning: using GhostTalonSRX for motor " + name + " on subsystem " + subsystemName, false);
        return CtreMotorFactory.createGhostTalon();
    }

    public IMotorController getMotor(String subsystemName, String name, IMotorController master) { // TODO: optimize this method
        if (isImplemented(subsystemName) && master != null) {
            YamlConfig.SubsystemConfig subsystem = getSubsystem(subsystemName);
            if (isHardwareValid(subsystem.talons.get(name))) {
                // Talons must be following another Talon, cannot follow a Victor.
                var talon = CtreMotorFactory.createPermanentSlaveTalon(subsystem.talons.get(name), false, master);
                talon.setInverted(master.getInverted());
                return talon;
            } else if (isHardwareValid(subsystem.falcons.get(name))) {
                var falcon = CtreMotorFactory.createPermanentSlaveTalon(subsystem.falcons.get(name), true, master);
                falcon.setInverted(master.getInverted());
                return falcon;
            } else if (isHardwareValid(subsystem.victors.get(name))) {
                // Victors can follow Talons or another Victor.
                var victor = CtreMotorFactory.createPermanentSlaveVictor(subsystem.victors.get(name), master);
                victor.setInverted(master.getInverted());
                return victor;
            }
        }
        DriverStation.reportWarning("Warning: using GhostTalonSRX for motor " + name + " on subsystem " + subsystemName, false);
        return CtreMotorFactory.createGhostTalon();
    }

    private boolean isHardwareValid(Integer hardwareId) {
        return hardwareId != null && hardwareId > -1;
    }

    public Solenoid getSolenoid(String subsystem, String name) {
        Integer solenoidId = getSubsystem(subsystem).solenoids.get(name);
        if (isHardwareValid(solenoidId)) {
            return new Solenoid(config.pcm, solenoidId);
        }
        return null;
    }

    public DoubleSolenoid getDoubleSolenoid(String subsystem, String name) {
        YamlConfig.DoubleSolenoidConfig solenoidConfig = getSubsystem(subsystem).doublesolenoids.get(name);
        if (solenoidConfig != null && isHardwareValid(solenoidConfig.forward) && isHardwareValid(solenoidConfig.reverse)) {
            return new DoubleSolenoid(config.pcm, solenoidConfig.forward, solenoidConfig.reverse);
        }
        return null;
    }

    public CANifier getCanifier(String subsystem) {
        if (isImplemented(subsystem) && getSubsystem(subsystem).canifier != null) {
            return new CANifier(getSubsystem(subsystem).canifier);
        }
        return null;
    }

    public Double getConstant(String name) {
        if (!config.constants.containsKey(name)) {
            DriverStation.reportError("Yaml constants:" + name + " missing", false);
            return null;
        }
        return config.constants.get(name);
    }

    public double getConstant(String subsystem, String name) {
        return getConstant(subsystem, name, -1);
    }

    public double getConstant(String subsystem, String name, double defaultVal) {
        if (getSubsystem(subsystem) == null) {
            DriverStation.reportError("Subsystem " + subsystem + " does not exist", false);
            return defaultVal;
        }
        if (!getSubsystem(subsystem).constants.containsKey(name)) {
            DriverStation.reportError("Yaml " + subsystem + " constants:" + name + " missing", false);
            return defaultVal;
        }
        return getSubsystem(subsystem).constants.get(name);
    }

    public YamlConfig getConfig() {
        return config;
    }

    public int getPcmId() {
        return config.pcm;
    }

    public YamlConfig.SubsystemConfig getSubsystem(String subsystem) {
        return config.subsystems.get(subsystem);
    }

    public static boolean isVerbose() {
        return verbose;
    }
}
