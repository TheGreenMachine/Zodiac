package com.team1816.lib.hardware;

import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.representer.Representer;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Since the Collections of configurations are injected by SnakeYaml,
// IDEs will report that the collections are never updated.
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class YamlConfig {
    private boolean $abstract = false;
    private String $extends;
    Map<String, SubsystemConfig> subsystems;
    Map<String, Double> constants = new HashMap<>();
    Integer pcm;

    public static YamlConfig loadFrom(InputStream input) throws ConfigIsAbstractException {
        Representer representer = new Representer();
        representer.getPropertyUtils().setSkipMissingProperties(true);
        Yaml yaml = new Yaml(new Constructor(YamlConfig.class), representer);
        yaml.setBeanAccess(BeanAccess.FIELD);

        YamlConfig loadedConfig = yaml.load(input);
        if (loadedConfig.$abstract) {
            throw new ConfigIsAbstractException();
        }

        if (loadedConfig.$extends != null && !loadedConfig.$extends.equals("")) {
            var baseConfigFile = YamlConfig.class.getClassLoader().getResourceAsStream(loadedConfig.$extends + ".config.yml");
            return merge(loadedConfig, loadRaw(baseConfigFile));
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
        boolean implemented = false;
        Map<String, Integer> talons = new HashMap<>();
        Map<String, Integer> falcons = new HashMap<>();
        Map<String, Integer> victors = new HashMap<>();
        Map<String, Integer> solenoids = new HashMap<>();
        Map<String, DoubleSolenoidConfig> doublesolenoids = new HashMap<>();
        Map<String, Double> constants = new HashMap<>();
        Integer canifier;
        List<String> invertMotor = new ArrayList<>();

        @Override
        public String toString() {
            return "SubsystemConfig {\n" +
                    "  implemented = " + implemented + ",\n" +
                    "  talons = " + talons.toString() + ",\n" +
                    "  falcons = " + falcons.toString() + ", \n" +
                    "  victors = " + victors.toString() + ",\n" +
                    "  invertMotor = " + invertMotor + ",\n" +
                    "  solenoids = " + solenoids.toString() + ",\n" +
                    "  doublesolenoids = " + doublesolenoids.toString() + ",\n" +
                    "  canifier = " + canifier + ",\n" +
                    "  constants = " + constants.toString() + ",\n" +
                    "}";
        }

        public static SubsystemConfig merge(SubsystemConfig active, SubsystemConfig base) {
            var result = new SubsystemConfig();
            if(active.implemented = true)
            {
                result.implemented = active.implemented;
            }
            else
            {
                result.implemented = base.implemented;
            }
            mergeMap(result.talons, active.talons, base.talons);
            mergeMap(result.falcons, active.falcons, base.falcons);
            mergeMap(result.victors, active.victors, base.victors);
            mergeMap(result.solenoids, active.solenoids, base.solenoids);
            mergeMap(result.doublesolenoids, active.doublesolenoids, base.doublesolenoids);
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
        public String toString() {
            return String.format("{ forward = %d, reverse = %d }", forward, reverse);
        }
    }

    @Override
    public String toString() {
        return "YamlConfig {\n" +
            "  subsystems = " + subsystems.toString() +
                "\n  pcm = " + pcm + "\n  constants = " + constants.toString() + "\n}";
    }

    public static YamlConfig merge(YamlConfig active, YamlConfig base) {
        var result = new YamlConfig();
        result.subsystems = new HashMap<>(base.subsystems);

        // Complex merge, add all subsystems in active config to result (already containing base subsystems)
        // using SubsystemConfig::merge to resolve conflicts.
        active.subsystems.forEach((key, value) ->
            result.subsystems.merge(key, value, (b, a) -> SubsystemConfig.merge(a, b)));
        mergeMap(result.constants, active.constants, base.constants);
        result.pcm = active.pcm != null ? active.pcm : base.pcm;
        result.$abstract = false;
        result.$extends = null;
        return result;
    }

    private static <K, V> void mergeMap(Map<K, V> result, Map<K, V> active, Map<K, V> fallback) {
        result.putAll(fallback);
        result.putAll(active);
    }
}
