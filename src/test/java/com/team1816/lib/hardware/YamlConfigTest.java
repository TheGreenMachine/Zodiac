package com.team1816.lib.hardware;

import static org.junit.Assert.*;

import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import com.team1816.frc2020.Robot;
import org.junit.Test;

public class YamlConfigTest {

    private YamlConfig loadConfig(String configName) {
        InputStream configFile = getClass()
            .getClassLoader()
            .getResourceAsStream(configName + ".config.yml");
        return YamlConfig.loadRaw(configFile);
    }

    @Test
    public void subsystemConfig_merge() {
        var base = loadConfig("test_base").subsystems.get("turret");
        var active = loadConfig("test_active").subsystems.get("turret");
        YamlConfig.SubsystemConfig result = YamlConfig.SubsystemConfig.merge(
            active,
            base
        );
        System.out.println(result);

        assertEquals("Base constant kP == 2.83", 2.83, result.constants.get("kP"), 0);
        assertEquals(
            "Overridden constant minPos == -374",
            -374,
            result.constants.get("minPos").intValue()
        );
        assertEquals(
            "New constant newConstant == 34",
            34,
            result.constants.get("newConstant"),
            0
        );
        assertEquals(
            "Turret talon should be overridden to 13",
            13,
            result.talons.get("turret").intValue()
        );
        assertTrue("implemented == true (favors true)", result.isImplemented());
    }

    @Test(expected = ConfigIsAbstractException.class)
    public void loadFromBase_throwsIfAbstract() throws ConfigIsAbstractException {
        YamlConfig.loadFrom(
            getClass().getClassLoader().getResourceAsStream("test_base.config.yml")
        );
    }

    @Test
    public void loadFromActive_doesNotThrow() throws ConfigIsAbstractException {
        YamlConfig.loadFrom(
            getClass().getClassLoader().getResourceAsStream("test_active.config.yml")
        );
    }

    @Test
    public void yamlConfig_merge() {
        YamlConfig base = loadConfig("test_base");
        YamlConfig active = loadConfig("test_active");
        YamlConfig result = YamlConfig.merge(active, base);

        verifyMergedConfig(result);
    }

    @Test
    public void yamlConfig_autoMerge_ifExtends() throws ConfigIsAbstractException {
        var configFile = getClass()
            .getClassLoader()
            .getResourceAsStream("test_active.config.yml");
        YamlConfig config = YamlConfig.loadFrom(configFile);
        verifyMergedConfig(config);
    }

    @Test
    public void testImplementedOverride() {
        mergeImplemented(true, false, true);
        mergeImplemented(false, true, false);
        mergeImplemented(true, true, true);
        mergeImplemented(false, false, false);
        mergeImplemented(null, null, false);
        mergeImplemented(null, true, true);
        mergeImplemented(true, null, true);
    }

    @Test
    public void outputMergedYaml() throws ConfigIsAbstractException, IOException {
        var configName = "zodiac_pro";
        InputStream configFile = Robot.class
            .getClassLoader()
            .getResourceAsStream(configName + ".config.yml");
        try (var writer = new FileWriter(configName + "_check.config.yml")) {
            writer.write(YamlConfig.loadFrom(configFile).toString());
        }
    }

    @Test
    public void verifyNewYaml_zodiacPro() throws ConfigIsAbstractException {
        var configName = "zodiac_pro";
        var newConfigFile = Robot.class
            .getClassLoader()
            .getResourceAsStream(configName + ".config.yml");
        var oldConfigFile = getClass()
            .getClassLoader()
            .getResourceAsStream(configName + "_check.config.yml");
        var oldConfig = YamlConfig.loadFrom(oldConfigFile);
        var newConfig = YamlConfig.loadFrom(newConfigFile);

        assertEquals("New config == old config", oldConfig.toString(), newConfig.toString());
    }

    private void mergeImplemented(Boolean active, Boolean base, boolean result) {
        var configActive = new YamlConfig.SubsystemConfig(active);
        var configBase = new YamlConfig.SubsystemConfig(base);
        var configResult = YamlConfig.SubsystemConfig.merge(configActive, configBase);
        assertEquals(result, configResult.isImplemented());
    }

    void verifyMergedConfig(YamlConfig config) {
        System.out.println(config);

        assertNotNull("Merged YAML config is not null", config);
        assertNotNull(
            "Subsystem config drivetrain is present",
            config.subsystems.get("drivetrain")
        );
        assertNotNull(
            "Subsystem config shooter is present",
            config.subsystems.get("shooter")
        );

        assertEquals(
            "Turret Talon ID == 13",
            13,
            config.subsystems.get("turret").talons.get("turret").intValue()
        );
        assertEquals(
            "Overridden constant turret.minPos == -374",
            -374,
            config.subsystems.get("turret").constants.get("minPos").intValue()
        );

        assertEquals(
            "Constant defined in base configuration baseConstant == 1",
            1,
            config.constants.get("baseConstant"),
            0
        );
        assertEquals(
            "Constant overridden in active config overriddenConstant == 0",
            0,
            config.constants.get("overriddenConstant"),
            0
        );
        assertEquals(
            "Constant defined in active configuration activeConstant == 399.42",
            399.42,
            config.constants.get("activeConstant"),
            0
        );

        assertEquals("PCM ID is 8", 8, config.pcm.intValue());
        assertTrue(
            "invertMotor for invertMotorTest subsystem contains motorA and motorB",
            config.subsystems
                .get("invertMotorTest")
                .invertMotor.containsAll(List.of("motorA", "motorB"))
        );
    }
}
