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
        try {
            config = YamlConfig.loadFrom(this.getClass().getClassLoader().getResourceAsStream(configName + ".config.yml"));
        } catch (ConfigIsAbstractException e) {
            DriverStation.reportError("Yaml Config was abstract!", e.getStackTrace());
        }
        verbose = getConstant("verbose") >= 1;
    }

    public boolean isImplemented(String subsystem) {
        return (getSubsystem(subsystem) != null) && (getSubsystem(subsystem).implemented);
    }

    public IMotorControllerEnhanced getMotor(String subsystemName, String name) {
        IMotorControllerEnhanced motor = null;
        YamlConfig.SubsystemConfig subsystem = getSubsystem(subsystemName);

        // Motor creation
        if (isImplemented(subsystemName)) {
            if (isHardwareValid(subsystem.talons.get(name))) {
                motor = CtreMotorFactory.createDefaultTalon(subsystem.talons.get(name), false);
            } else if (isHardwareValid(subsystem.falcons.get(name))) {
                motor = CtreMotorFactory.createDefaultTalon(subsystem.falcons.get(name), true);
            } // Never make the victor a master
        }
        if (motor == null) {
            DriverStation.reportWarning("Warning: using GhostTalonSRX for motor " + name + " on subsystem " + subsystemName, false);
            motor = CtreMotorFactory.createGhostTalon();
        }

        // Motor configuration
        if (subsystem != null && subsystem.invertMotor.contains(name)) {
            System.out.println("Inverting " + name + " with ID " + motor.getDeviceID());
            motor.setInverted(true);
        }
        motor.config_kP(0, getConstant(subsystemName, "kP", 0), Constants.kLongCANTimeoutMs);
        motor.config_kI(0, getConstant(subsystemName, "kI", 0), Constants.kLongCANTimeoutMs);
        motor.config_kD(0, getConstant(subsystemName, "kD", 0), Constants.kLongCANTimeoutMs);
        motor.config_kF(0, getConstant(subsystemName, "kF", 0), Constants.kLongCANTimeoutMs);

        return motor;
    }

    public IMotorController getMotor(String subsystemName, String name, IMotorController master) { // TODO: optimize this method
        IMotorController motor = null;
        if (isImplemented(subsystemName) && master != null) {
            YamlConfig.SubsystemConfig subsystem = getSubsystem(subsystemName);
            if (isHardwareValid(subsystem.talons.get(name))) {
                // Talons must be following another Talon, cannot follow a Victor.
                motor = CtreMotorFactory.createPermanentSlaveTalon(subsystem.talons.get(name), false, master);
            } else if (isHardwareValid(subsystem.falcons.get(name))) {
                motor = CtreMotorFactory.createPermanentSlaveTalon(subsystem.falcons.get(name), true, master);
            } else if (isHardwareValid(subsystem.victors.get(name))) {
                // Victors can follow Talons or another Victor.
                motor = CtreMotorFactory.createPermanentSlaveVictor(subsystem.victors.get(name), master);
            }
        }
        if (motor == null) {
            DriverStation.reportWarning("Warning: using GhostTalonSRX for motor " + name + " on subsystem " + subsystemName, false);
            motor = CtreMotorFactory.createGhostTalon();
        }
        if (master != null) {
            motor.setInverted(master.getInverted());
        }
        return motor;
    }

    private boolean isHardwareValid(Integer hardwareId) {
        return hardwareId != null && hardwareId > -1;
    }

    public Solenoid getSolenoid(String subsystem, String name) {
        Integer solenoidId = getSubsystem(subsystem).solenoids.get(name);
        if (isHardwareValid(solenoidId)) {
            return new Solenoid(config.pcm, solenoidId);
        }
        DriverStation.reportError(
            "Solenoid " + name +
                " not defined or invalid in config for subsystem " + subsystem, false);
        return null;
    }

    public DoubleSolenoid getDoubleSolenoid(String subsystem, String name) {
        YamlConfig.DoubleSolenoidConfig solenoidConfig = getSubsystem(subsystem).doublesolenoids.get(name);
        if (solenoidConfig != null && isHardwareValid(solenoidConfig.forward) && isHardwareValid(solenoidConfig.reverse)) {
            return new DoubleSolenoid(config.pcm, solenoidConfig.forward, solenoidConfig.reverse);
        }
        DriverStation.reportError(
            "DoubleSolenoid " + name +
                " not defined or invalid in config for subsystem " + subsystem, false);
        return null;
    }

    public CANifier getCanifier(String subsystem) {
        if (isImplemented(subsystem) && getSubsystem(subsystem).canifier != null) {
            return new CANifier(getSubsystem(subsystem).canifier);
        }
        DriverStation.reportError("CANifier ID not defined for subsystem "
            + subsystem + "! CANifier will be NULL!", false);
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
