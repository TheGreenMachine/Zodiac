package com.team1816.lib.hardware;

import java.util.*;

public class SubsystemConfig {

    private Boolean implemented;

    // Motors
    Map<String, Integer> talons = new HashMap<>();
    Map<String, Integer> falcons = new HashMap<>();
    Map<String, Integer> victors = new HashMap<>();
    List<String> invertMotor = new ArrayList<>();
    Map<String, SwerveModuleConfig> swerveModules = new HashMap<>();

    // Pneumatics
    Map<String, Integer> solenoids = new HashMap<>();
    Map<String, DoubleSolenoidConfig> doubleSolenoids = new HashMap<>();

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
        SubsystemConfig that = (SubsystemConfig) o;
        return (
            Objects.equals(implemented, that.implemented) &&
                talons.equals(that.talons) &&
                falcons.equals(that.falcons) &&
                victors.equals(that.victors) &&
                solenoids.equals(that.solenoids) &&
                doubleSolenoids.equals(that.doubleSolenoids) &&
                constants.equals(that.constants) &&
                Objects.equals(canifier, that.canifier) &&
                invertMotor.equals(that.invertMotor)
        );
    }

    @Override
    public int hashCode() {
        return Objects.hash(
            implemented,
            talons,
            falcons,
            victors,
            solenoids,
            doubleSolenoids,
            constants,
            canifier,
            invertMotor
        );
    }

    @Override
    public String toString() {
        return YamlConfig.FORMATTER.dump(this);
    }

    public static SubsystemConfig merge(
        SubsystemConfig active,
        SubsystemConfig base
    ) {
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
        YamlConfig.mergeMap(result.solenoids, active.solenoids, base.solenoids);
        YamlConfig.mergeMap(
            result.doubleSolenoids,
            active.doubleSolenoids,
            base.doubleSolenoids
        );
        result.invertMotor.addAll(base.invertMotor);
        result.invertMotor.addAll(active.invertMotor);
        result.canifier = active.canifier != null ? active.canifier : base.canifier;
        YamlConfig.mergeMap(result.constants, active.constants, base.constants);

        return result;
    }
}
