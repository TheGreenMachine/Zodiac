package com.team1816.lib.hardware;

import java.io.InputStream;
import java.util.*;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.representer.Representer;

// Since the Collections of configurations are injected by SnakeYaml,
// IDEs will report that the collections are never updated.
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class YamlConfig {

    private static final Yaml FORMATTER = new Yaml();

    static {
        FORMATTER.setBeanAccess(BeanAccess.FIELD);
    }

    private boolean $abstract = false;
    private String $extends;
    Map<String, SubsystemConfig> subsystems;
    Map<String, Double> constants = new HashMap<>();
    Integer pcm;

    public static YamlConfig loadFrom(InputStream input)
        throws ConfigIsAbstractException {
        YamlConfig loadedConfig = loadInternal(input);
        if (loadedConfig.$abstract) {
            throw new ConfigIsAbstractException();
        }

        return loadedConfig;
    }

    static YamlConfig loadInternal(InputStream input) {
        YamlConfig loadedConfig = loadRaw(input);

        if (loadedConfig.$extends != null && !loadedConfig.$extends.equals("")) {
            var baseConfigFile =
                YamlConfig.class.getClassLoader()
                    .getResourceAsStream(loadedConfig.$extends + ".config.yml");
            return merge(loadedConfig, loadInternal(baseConfigFile));
        }

        return loadedConfig;
    }

    static YamlConfig loadRaw(InputStream input) {
        Representer representer = new Representer();
        representer.getPropertyUtils().setSkipMissingProperties(true);
        Yaml yaml = new Yaml(new Constructor(YamlConfig.class), representer);
        yaml.setBeanAccess(BeanAccess.FIELD);

        return yaml.load(input);
    }

    public Double getConstant(String key) {
        return constants.get(key);
    }

    public static class SubsystemConfig {

        private Boolean implemented;
        Map<String, Integer> talons = new HashMap<>();
        Map<String, Integer> falcons = new HashMap<>();
        Map<String, Integer> victors = new HashMap<>();
        Map<String, Integer> solenoids = new HashMap<>();
        Map<String, DoubleSolenoidConfig> doublesolenoids = new HashMap<>();
        Map<String, Double> constants = new HashMap<>();
        Integer canifier;
        List<String> invertMotor = new ArrayList<>();

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
                doublesolenoids.equals(that.doublesolenoids) &&
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
                doublesolenoids,
                constants,
                canifier,
                invertMotor
            );
        }

        @Override
        public String toString() {
            return FORMATTER.dump(this);
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

            mergeMap(result.talons, active.talons, base.talons);
            mergeMap(result.falcons, active.falcons, base.falcons);
            mergeMap(result.victors, active.victors, base.victors);
            mergeMap(result.solenoids, active.solenoids, base.solenoids);
            mergeMap(
                result.doublesolenoids,
                active.doublesolenoids,
                base.doublesolenoids
            );
            result.invertMotor.addAll(base.invertMotor);
            result.invertMotor.addAll(active.invertMotor);
            result.canifier = active.canifier != null ? active.canifier : base.canifier;
            mergeMap(result.constants, active.constants, base.constants);

            return result;
        }
    }

    public static class DoubleSolenoidConfig {

        int forward;
        int reverse;

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            DoubleSolenoidConfig that = (DoubleSolenoidConfig) o;
            return forward == that.forward && reverse == that.reverse;
        }

        @Override
        public int hashCode() {
            return Objects.hash(forward, reverse);
        }

        @Override
        public String toString() {
            return String.format("{ forward = %d, reverse = %d }", forward, reverse);
        }
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        YamlConfig that = (YamlConfig) o;
        return (
            $abstract == that.$abstract &&
            Objects.equals($extends, that.$extends) &&
            subsystems.equals(that.subsystems) &&
            constants.equals(that.constants) &&
            Objects.equals(pcm, that.pcm)
        );
    }

    @Override
    public int hashCode() {
        return Objects.hash($abstract, $extends, subsystems, constants, pcm);
    }

    @Override
    public String toString() {
        return FORMATTER.dump(this);
    }

    public static YamlConfig merge(YamlConfig active, YamlConfig base) {
        var result = new YamlConfig();
        result.subsystems = new HashMap<>(base.subsystems);

        // Complex merge, add all subsystems in active config to result (already containing base subsystems)
        // using SubsystemConfig::merge to resolve conflicts.
        active.subsystems.forEach(
            (key, value) ->
                result.subsystems.merge(key, value, (b, a) -> SubsystemConfig.merge(a, b))
        );
        mergeMap(result.constants, active.constants, base.constants);
        result.pcm = active.pcm != null ? active.pcm : base.pcm;
        result.$abstract = active.$abstract;
        result.$extends = null;
        return result;
    }

    private static <K, V> void mergeMap(
        Map<K, V> result,
        Map<K, V> active,
        Map<K, V> fallback
    ) {
        result.putAll(fallback);
        result.putAll(active);
    }
}
