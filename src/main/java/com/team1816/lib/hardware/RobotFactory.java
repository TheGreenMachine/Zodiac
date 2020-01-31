package com.team1816.lib.hardware;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;

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

    public IMotorControllerEnhanced getMotor(String subsystemName, String name) {
        if (isImplemented(subsystemName)) {
            YamlConfig.SubsystemConfig subsystem = getSubsystem(subsystemName);
            if (isHardwareValid(subsystem.talons.get(name))) {
                var motor = CtreMotorFactory.createDefaultTalon(subsystem.talons.get(name));
                if (subsystem.invertMotor.contains(motor.getDeviceID())) {
                    System.out.println("Inverting " + name);
                    motor.setInverted(true);
                }
                return motor;
            } else if (isHardwareValid(subsystem.falcons.get(name))) {
                var motor = CtreMotorFactory.createDefaultFalcon(subsystem.falcons.get(name));
                if (subsystem.invertMotor.contains(motor.getDeviceID())) {
                    System.out.println("Inverting" + name);
                    motor.setInverted(true);
                }
                return motor;
            } // Never make the victor a master
        }
        return CtreMotorFactory.createGhostTalon();
    }

    public IMotorController getMotor(String subsystemName, String name, IMotorController master) {
        if (isImplemented(subsystemName) && master != null) {
            YamlConfig.SubsystemConfig subsystem = getSubsystem(subsystemName);
            if (isHardwareValid(subsystem.talons.get(name))) {
                // Talons must be following another Talon, cannot follow a Victor.
                var talon = CtreMotorFactory.createPermanentSlaveTalon(subsystem.talons.get(name), master);
                talon.setInverted(master.getInverted());
                return talon;
            } else if (isHardwareValid(subsystem.victors.get(name))) {
                // Victors can follow Talons or another Victor.
                var victor = CtreMotorFactory.createPermanentSlaveVictor(subsystem.victors.get(name), master);
                victor.setInverted(master.getInverted());
                return victor;
            }
        }
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

    public Double getConstant(String subsystem, String name) {
        if (!getSubsystem(subsystem).constants.containsKey(name)) {
            DriverStation.reportError("Yaml " + subsystem + " constants:" + name + " missing", false);
            return null;
        }
        return getSubsystem(subsystem).constants.get(name);
    }

    public YamlConfig getConfig() {
        return config;
    }

    public YamlConfig.SubsystemConfig getSubsystem(String subsystem) {
        return config.subsystems.get(subsystem);
    }

    public static boolean Verbose() {
        return verbose;
    }
}
