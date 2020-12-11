package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Constants;
import com.team1816.lib.hardware.components.CanifierImpl;
import com.team1816.lib.hardware.components.GhostCanifier;
import com.team1816.lib.hardware.components.ICanifier;
import com.team1816.lib.hardware.components.pcm.*;
import edu.wpi.first.wpilibj.DriverStation;

import javax.annotation.Nonnull;

public class RobotFactory {

    private YamlConfig config;
    private static boolean verbose;
    private static RobotFactory factory;

    public static RobotFactory getInstance() {
        if (factory == null) {
            var robotName = System.getenv("ROBOT_NAME");
            if (robotName == null) {
                robotName = "default";
                DriverStation.reportWarning("ROBOT_NAME environment variable not defined, falling back to default.config.yml!", false);
            }
            factory = new RobotFactory(robotName);
        }
        return factory;
    }

    public RobotFactory(String configName) {
        System.out.println("Loading Config for " + configName);
        try {
            config = YamlConfig.loadFrom(this.getClass().getClassLoader().getResourceAsStream(configName + ".config.yml"));
        } catch (ConfigIsAbstractException e) {
            DriverStation.reportError("Yaml Config was abstract!", e.getStackTrace());
        }
        verbose = getConstant("verbose") >= 1;
    }

    public IMotorControllerEnhanced getMotor(String subsystemName, String name) {
        IMotorControllerEnhanced motor = null;
        var subsystem = getSubsystem(subsystemName);

        // Motor creation
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.talons.get(name))) {
                motor = CtreMotorFactory.createDefaultTalon(subsystem.talons.get(name), false);
            } else if (isHardwareValid(subsystem.falcons.get(name))) {
                motor = CtreMotorFactory.createDefaultTalon(subsystem.falcons.get(name), true);
            } // Never make the victor a master
        }
        if (motor == null) {
            if(subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
            motor = CtreMotorFactory.createGhostTalon();
        }

        // Motor configuration
        if (subsystem.implemented && subsystem.invertMotor.contains(name)) {
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
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && master != null) {
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
            if(subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
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

    @Nonnull
    public ISolenoid getSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        Integer solenoidId = subsystem.solenoids.get(name);
        if (subsystem.implemented && isHardwareValid(solenoidId)) {
            return new SolenoidImpl(config.pcm, solenoidId);
        }
        if(subsystem.implemented) {
            reportGhostWarning("Solenoid", subsystemName, name);
        }
        return new GhostSolenoid();
    }

    @Nonnull
    public IDoubleSolenoid getDoubleSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        YamlConfig.DoubleSolenoidConfig solenoidConfig = getSubsystem(subsystemName).doublesolenoids.get(name);
        if (
            subsystem.implemented
            && solenoidConfig != null
            && isHardwareValid(solenoidConfig.forward)
            && isHardwareValid(solenoidConfig.reverse)
        ) {
            return new DoubleSolenoidImpl(config.pcm, solenoidConfig.forward, solenoidConfig.reverse);
        }
        reportGhostWarning("DoubleSolenoid", subsystemName, name);
        return new GhostDoubleSolenoid();
    }

    @Nonnull
    public ICanifier getCanifier(String subsystemName) {
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && isHardwareValid(subsystem.canifier)) {
            return new CanifierImpl(subsystem.canifier);
        }
        reportGhostWarning("CANifier", subsystemName, "canifier");
        return new GhostCanifier();
    }

    public Double getConstant(String name) {
        return getConstant(name,0.0);
    }

    public Double getConstant(String name, double defaultVal) {
        if (!config.constants.containsKey(name)) {
            DriverStation.reportError("Yaml constants:" + name + " missing", false);
            return defaultVal;
        }
        return config.constants.get(name);
    }

    public double getConstant(String subsystemName, String name) {
        return getConstant(subsystemName, name, 0.0);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        if (!getSubsystem(subsystemName).implemented) {
            return defaultVal;
        }
        if (!getSubsystem(subsystemName).constants.containsKey(name)) {
            DriverStation.reportError("Yaml " + subsystemName + " constants:" + name + " missing", false);
            return defaultVal;
        }
        return getSubsystem(subsystemName).constants.get(name);
    }

    public int getPcmId() {
        if(config.pcm == null) return -1;
        return config.pcm;
    }

    public YamlConfig.SubsystemConfig getSubsystem(String subsystemName) {
        var subsystem = config.subsystems.get(subsystemName);
        if(subsystem == null) {
            subsystem = new YamlConfig.SubsystemConfig();
            subsystem.implemented = false;
        }
        return subsystem;
    }

    public static boolean isVerbose() {
        return verbose;
    }

    private void reportGhostWarning(String type, String subsystemName, String componentName) {
        DriverStation.reportWarning(
            type + "  " + componentName
                + " not defined or invalid in config for subsystem " + subsystemName
                + ", using ghost!", false);
    }
}
