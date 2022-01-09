package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.subsystems.SwerveModule;
import com.team1816.lib.hardware.components.CanifierImpl;
import com.team1816.lib.hardware.components.GhostCanifier;
import com.team1816.lib.hardware.components.ICanifier;
import com.team1816.lib.hardware.components.pcm.*;
import com.team1816.lib.subsystems.Subsystem;
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
        try {
            config =
                YamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(configName + ".config.yml")
                );
        } catch (Exception e) {
            DriverStation.reportError("Yaml Config error!", e.getStackTrace());
        }
        verbose = getConstant("verbose") >= 1;
    }

    public IMotorControllerEnhanced getMotor(
        String subsystemName,
        String name,
        List<PIDSlotConfiguration> pidConfigs
    ) {
        IMotorControllerEnhanced motor = null;
        var subsystem = config.getSubsystems().get(subsystemName);

        // Motor creation
        if (subsystem.getImplemented()) {
            if (isHardwareValid(subsystem.getTalons().get(name))) {
                motor =
                    CtreMotorFactory.createDefaultTalon(
                        subsystem.getTalons().get(name),
                        name,
                        false,
                        subsystem,
                        pidConfigs
                    );
            } else if (isHardwareValid(subsystem.getFalcons().get(name))) {
                motor =
                    CtreMotorFactory.createDefaultTalon(
                        subsystem.getFalcons().get(name),
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
        return getMotor(subsystemName, name, getSubsystem(subsystemName).getPidConfig());
    }

    public IMotorController getMotor(
        String subsystemName,
        String name,
        IMotorController master
    ) { // TODO: optimize this method
        IMotorController followerMotor = null;
        var subsystem = config.getSubsystems().get(subsystemName);
        if (subsystem.getImplemented() && master != null) {
            if (isHardwareValid(subsystem.getTalons().get(name))) {
                // Talons must be following another Talon, cannot follow a Victor.
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveTalon(
                        subsystem.getFalcons().get(name),
                        name,
                        false,
                        master,
                        subsystem,
                        subsystem.getPidConfig()
                    );
            } else if (isHardwareValid(subsystem.getFalcons().get(name))) {
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveTalon(
                        subsystem.getFalcons().get(name),
                        name,
                        true,
                        master,
                        subsystem,
                        subsystem.getPidConfig()
                    );
            } else if (isHardwareValid(subsystem.getVictors().get(name))) {
                // Victors can follow Talons or another Victor.
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveVictor(
                        subsystem.getVictors().get(name),
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
        var subsystem = config.getSubsystems().get(subsystemName);
        SwerveModuleConfiguration module = subsystem.getSwerveModules().get(name);
        if (module == null) {
            DriverStation.reportError(
                "No swerve module with name " + name + " subsystem " + subsystemName,
                true
            );
            return null;
        }

        var swerveConstants = new SwerveModule.SwerveModuleConstants();
        swerveConstants.kName = name;
        swerveConstants.kAzimuthMotorName = module.getModules().get(name).getAzimuth(); //getAzimuth and drive give ID i think - not the module name (ex: leftRear)
        swerveConstants.kAzimuthPid = subsystem.getSwerveModules().get(name).getAzimuthPID().get(0);
        swerveConstants.kDriveMotorName = module.getModules().get(name).getDrive();
        swerveConstants.kDrivePid = subsystem.getSwerveModules().get(name).getDrivePID().get(0);
        swerveConstants.kAzimuthEncoderHomeOffset = module.getModules().get(name).getConstants().get("encoderOffset");
        swerveConstants.kInvertAzimuthSensorPhase = module.getConstants().get("invertedSensorPhase")==1; //boolean

        return new SwerveModule(subsystemName, swerveConstants, startPos);
    }

    @Nonnull
    public ISolenoid getSolenoid(String subsystemName, String name) {
        var subsystem = config.getSubsystems().get(subsystemName);
        Integer solenoidId = subsystem.getSolenoids().get(name);
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
        var subsystem = config.getSubsystems().get(subsystemName);
        DoubleSolenoidConfig solenoidConfig = getSubsystem(subsystemName)
            .getDoubleSolenoids().get(name);
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
        var subsystem = config.getSubsystems().get(subsystemName);
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
        return config.getConstants();
    }

    public SubsystemConfig getSubsystem(String key){
        var subsystem = config.getSubsystems().get(key);
        if(subsystem == null) System.out.println("Subsystem not defined: " + key);
        return subsystem;
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
        if (!getSubsystem(subsystemName).getConstants().containsKey(name)) {
            DriverStation.reportError(
                "Yaml " + subsystemName + " constants:" + name + " missing",
                false
            );
            return defaultVal;
        }
        return getSubsystem(subsystemName).getConstants().get(name);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName, int slot) {
        return getSubsystem(subsystemName).getPidConfig().get(0);
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
