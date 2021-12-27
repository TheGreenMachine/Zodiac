package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.subsystems.SwerveModule;
import com.team1816.lib.hardware.components.CanifierImpl;
import com.team1816.lib.hardware.components.GhostCanifier;
import com.team1816.lib.hardware.components.ICanifier;
import com.team1816.lib.hardware.components.pcm.*;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import java.util.List;
import java.util.Map;
import javax.annotation.Nonnull;

public class RobotFactory {

    private RobotConfiguration config;
    private static boolean verbose;
    private static RobotFactory factory;

    public static RobotFactory getInstance() {
        if (factory == null) {
            var robotName = System.getenv("ROBOT_NAME");
            if (robotName == null) {
                robotName = "default";
                DriverStation.reportWarning(
                    "ROBOT_NAME environment variable not defined, falling back to default.config.yml!",
                    false
                );
            }
            factory = new RobotFactory(robotName);
        }
        return factory;
    }

    public RobotFactory(String configName) {
        System.out.println("Loading Config for " + configName);
            config =
                YamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(configName + ".config.yml")
                );
        verbose = getConstant("verbose") >= 1;
    }

    public IMotorControllerEnhanced getMotor(
        String subsystemName,
        String name,
        List<PIDConfiguration> pidConfigs
    ) {
        IMotorControllerEnhanced motor = null;
        var subsystem = getSubsystem(subsystemName);

        // Motor creation
        if (subsystem.getImplemented()) {
            if (isHardwareValid(subsystem.getTalons().getAdditionalProperties().get(name))) {
                motor =
                    CtreMotorFactory.createDefaultTalon(
                        subsystem.getTalons().getAdditionalProperties().get(name),
                        name,
                        false,
                        subsystem,
                        pidConfigs
                    );
            } else if (isHardwareValid(subsystem.getFalcons().getAdditionalProperties().get(name))) {
                motor =
                    CtreMotorFactory.createDefaultTalon(
                        subsystem.getFalcons().getAdditionalProperties().get(name),
                        name,
                        true,
                        subsystem,
                        pidConfigs
                    );
            } // Never make the victor a master
        }
        if (motor == null) {
            reportGhostWarning("Motor", subsystemName, name);
            motor = CtreMotorFactory.createGhostTalon();
        }

        // Motor configuration
        if (subsystem.getImplemented() && subsystem.getInvertMotor().contains(name)) {
            System.out.println("Inverting " + name + " with ID " + motor.getDeviceID());
            motor.setInverted(true);
        }
        if (subsystem.getImplemented() && subsystem.getInvertSensorPhase().contains(name)) {
            System.out.println("Inverting sensor phase of " + name + " with ID " + motor.getDeviceID());
            motor.setSensorPhase(true);
        }

        return motor;
    }

    public IMotorControllerEnhanced getMotor(String subsystemName, String name) {
        return getMotor(subsystemName, name, getSubsystem(subsystemName).getPid());
    }

    public IMotorController getMotor(
        String subsystemName,
        String name,
        IMotorController master
    ) { // TODO: optimize this method
        IMotorController followerMotor = null;
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.getImplemented() && master != null) {
            if (isHardwareValid(subsystem.getTalons().getAdditionalProperties().get(name))) {
                // Talons must be following another Talon, cannot follow a Victor.
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveTalon(
                        subsystem.getFalcons().getAdditionalProperties().get(name),
                        name,
                        false,
                        master,
                        subsystem,
                        subsystem.getPid()
                    );
            } else if (isHardwareValid(subsystem.getFalcons().getAdditionalProperties().get(name))) {
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveTalon(
                        subsystem.getFalcons().getAdditionalProperties().get(name),
                        name,
                        true,
                        master,
                        subsystem,
                        subsystem.getPid()
                    );
            } else if (isHardwareValid(subsystem.getVictors().getAdditionalProperties().get(name))) {
                // Victors can follow Talons or another Victor.
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveVictor(
                        subsystem.getVictors().getAdditionalProperties().get(name),
                        master
                    );
            }
        }
        if (followerMotor == null) {
            if (subsystem.getImplemented()) reportGhostWarning(
                "Motor",
                subsystemName,
                name
            );
            followerMotor = CtreMotorFactory.createGhostTalon();
        }
        if (master != null) {
            followerMotor.setInverted(master.getInverted());
        }
        return followerMotor;
    }

    private boolean isHardwareValid(Integer hardwareId) {
        return hardwareId != null && hardwareId > -1;
    }

    public SwerveModule getSwerveModule(
        String subsystemName,
        String name,
        Translation2d startPos
    ) {
        var subsystem = getSubsystem(subsystemName);
        SwerveModuleConfiguration module = subsystem.getSwerveModules().getAdditionalProperties().get(name);
        if (module == null) {
            DriverStation.reportError(
                "No swerve module with name " + name + " subsystem " + subsystemName,
                true
            );
            return null;
        }

        var swerveConstants = new SwerveModule.SwerveModuleConstants();
        swerveConstants.kName = name;
        swerveConstants.kAzimuthMotorName = module.getAzimuth(); //getAzimuth and drive give ID i think - not the module name (ex: leftRear)
        swerveConstants.kAzimuthPid = subsystem.getAzimuthPid();
        swerveConstants.kDriveMotorName = module.getDrive();
        swerveConstants.kDrivePid = subsystem.getDrivePid();
        swerveConstants.kAzimuthEncoderHomeOffset = module.getEncoderOffset();
        swerveConstants.kInvertAzimuthSensorPhase = module.getInvertSensorPhase();

        return new SwerveModule(subsystemName, swerveConstants, startPos);
    }

    @Nonnull
    public ISolenoid getSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        Integer solenoidId = subsystem.getSolenoids().getAdditionalProperties().get(name);
        if (subsystem.getImplemented() && isHardwareValid(solenoidId) && isPcmEnabled()) {
            return new SolenoidImpl(config.getPcm(), PneumaticsModuleType.CTREPCM, solenoidId);
        }
        if (subsystem.getImplemented()) {
            reportGhostWarning("Solenoid", subsystemName, name);
        }
        return new GhostSolenoid();
    }

    @Nonnull
    public IDoubleSolenoid getDoubleSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        DoubleSolenoidsConfiguration solenoidConfig = getSubsystem(subsystemName)
            .getDoubleSolenoids().getAdditionalProperties().get(name);
        if (
            subsystem.getImplemented() &&
            solenoidConfig != null &&
            isHardwareValid(solenoidConfig.getForward()) &&
            isHardwareValid(solenoidConfig.getForward()) &&
            isPcmEnabled()
        ) {
            return new DoubleSolenoidImpl(
                config.getPcm(),
                PneumaticsModuleType.CTREPCM,
                solenoidConfig.getForward(),
                solenoidConfig.getReverse()
            );
        }
        reportGhostWarning("DoubleSolenoid", subsystemName, name);
        return new GhostDoubleSolenoid();
    }

    @Nonnull
    public ICanifier getCanifier(String subsystemName) {
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.getImplemented() && isHardwareValid(subsystem.getCanifier())) {
            return new CanifierImpl(subsystem.getCanifier());
        }
        reportGhostWarning("CANifier", subsystemName, "canifier");
        return new GhostCanifier();
    }

    public ICompressor getCompressor() {
        if (isPcmEnabled()) {
            return new CompressorImpl(getPcmId(), PneumaticsModuleType.CTREPCM);
        }
        reportGhostWarning("Compressor", "ROOT", "on PCM ID " + getPcmId());
        return new GhostCompressor();
    }

    public Double getConstant(String name) {
        return getConstant(name, 0.0);
    }

    public Map<String, Double> getConstants() {
        return config.getConstants().getAdditionalProperties();
    }

    public SubsystemConfiguration getSubsystem(String key){
        return config.getSubsystems().getAdditionalProperties().get(key);
    }

    public Double getConstant(String name, double defaultVal) {
        if (!getConstants().containsKey(name)) {
            DriverStation.reportError("Yaml constants:" + name + " missing", false);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public double getConstant(String subsystemName, String name) {
        return getConstant(subsystemName, name, 0.0);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        if (!getSubsystem(subsystemName).getImplemented()) {
            return defaultVal;
        }
        if (!getSubsystem(subsystemName).getConstants().getAdditionalProperties().containsKey(name)) {
            DriverStation.reportError(
                "Yaml " + subsystemName + " constants:" + name + " missing",
                false
            );
            return defaultVal;
        }
        return getSubsystem(subsystemName).getConstants().getAdditionalProperties().get(name);
    }

    public PIDConfiguration getPidConfig(String subsystemName, int slot) {
        return getSubsystem(subsystemName).getPid().get(slot);
    }

    public int getPcmId() {
        if (config.getPcm() == null) return -1;
        return config.getPcm();
    }

    public boolean isPcmEnabled() {
        return getPcmId() > -1;
    }

    public static boolean isVerbose() {
        return verbose;
    }

    private void reportGhostWarning(
        String type,
        String subsystemName,
        String componentName
    ) {
        System.out.println(
            "  " +
            type +
            "  " +
            componentName +
            " not defined or invalid in config for subsystem " +
            subsystemName +
            ", using ghost!"
        );
    }
}
