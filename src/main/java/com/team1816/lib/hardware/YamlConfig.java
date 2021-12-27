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

    SubsystemMap subsystems;
    HardwareConstants constants;
    Integer pcm;

    public static RobotConfiguration loadFrom(InputStream input){
        RobotConfiguration loadedConfig = loadInternal(input);

        return loadedConfig;
    }

    static RobotConfiguration loadInternal(InputStream input) {
        RobotConfiguration loadedConfig = loadRaw(input);

        return loadedConfig;
    }

    static RobotConfiguration loadRaw(InputStream input) {
        Representer representer = new Representer();
        representer.getPropertyUtils().setSkipMissingProperties(true);
        Yaml yaml = new Yaml(new Constructor(RobotConfiguration.class), representer);
        yaml.setBeanAccess(BeanAccess.FIELD);

        return yaml.load(input);
    }

    @Override
    public int hashCode() {
        return Objects.hash(subsystems, constants, pcm);
    }

    @Override
    public String toString() {
        return FORMATTER.dump(this);
    }

}
