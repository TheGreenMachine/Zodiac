package com.team1816.lib.hardware;

import java.util.*;

public class SubsystemConfig {

    private Boolean implemented;

    // Motors
    Map<String, Integer> talons = new HashMap<>();
    Map<String, Integer> falcons = new HashMap<>();
    Map<String, Integer> victors = new HashMap<>();
    List<String> invertMotor = new ArrayList<>();
    List<PidConfig> pid = new ArrayList<>();
    Map<String, SwerveModuleConfig> swerveModules = new HashMap<>();
    PidConfig drivePid;
    PidConfig azimuthPid;

    // Pneumatics
    Map<String, Integer> solenoids = new HashMap<>();
    Map<String, DoubleSolenoidConfig> doubleSolenoids = new HashMap<>();

    // Configuration
    Map<String, Double> constants = new HashMap<>();
    Integer canifier;

    public SubsystemConfig() {
        // no-op
    }

    public SubsystemConfig(Boolean implemented) {
        this.implemented = implemented;
    }

    public boolean isImplemented() {
        return Objects.requireNonNullElse(implemented, false);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SubsystemConfig subsystemConfig = (SubsystemConfig) o;
        return (
            Objects.equals(implemented, subsystemConfig.implemented) &&
            talons.equals(subsystemConfig.talons) &&
            falcons.equals(subsystemConfig.falcons) &&
            victors.equals(subsystemConfig.victors) &&
            swerveModules.equals(subsystemConfig.swerveModules) &&
            solenoids.equals(subsystemConfig.solenoids) &&
            doubleSolenoids.equals(subsystemConfig.doubleSolenoids) &&
            constants.equals(subsystemConfig.constants) &&
            Objects.equals(pid, subsystemConfig.pid) &&
            Objects.equals(azimuthPid, subsystemConfig.azimuthPid) &&
            Objects.equals(drivePid, subsystemConfig.drivePid) &&
            Objects.equals(canifier, subsystemConfig.canifier) &&
            invertMotor.equals(subsystemConfig.invertMotor)
        );
    }

    @Override
    public int hashCode() {
        return Objects.hash(
            implemented,
            talons,
            falcons,
            victors,
            swerveModules,
            solenoids,
            doubleSolenoids,
            constants,
            pid,
            azimuthPid,
            drivePid,
            canifier,
            invertMotor
        );
    }

    @Override
    public String toString() {
        return YamlConfig.FORMATTER.dump(this);
    }

    public static SubsystemConfig merge(SubsystemConfig active, SubsystemConfig base) {
        var result = new SubsystemConfig();

        if (active.implemented != null) {
            result.implemented = active.implemented;
        } else if (base.implemented != null) {
            result.implemented = base.implemented;
        } else {
            result.implemented = false;
        }

        YamlConfig.mergeMap(result.talons, active.talons, base.talons);
        YamlConfig.mergeMap(result.falcons, active.falcons, base.falcons);
        YamlConfig.mergeMap(result.victors, active.victors, base.victors);
        YamlConfig.mergeMap(
            result.swerveModules,
            active.swerveModules,
            base.swerveModules
        );
        YamlConfig.mergeMap(result.solenoids, active.solenoids, base.solenoids);
        YamlConfig.mergeMap(
            result.doubleSolenoids,
            active.doubleSolenoids,
            base.doubleSolenoids
        );
        result.invertMotor.addAll(base.invertMotor);
        result.invertMotor.addAll(active.invertMotor);
        if (active.pid.size() != 0) {
            result.pid = active.pid;
        } else {
            result.pid = base.pid;
        }

        result.azimuthPid =
            active.azimuthPid != null ? active.azimuthPid : base.azimuthPid;
        result.drivePid = active.drivePid != null ? active.drivePid : base.drivePid;
        result.canifier = active.canifier != null ? active.canifier : base.canifier;
        YamlConfig.mergeMap(result.constants, active.constants, base.constants);

        return result;
    }
}
