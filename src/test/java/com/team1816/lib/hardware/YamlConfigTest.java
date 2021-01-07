package com.team1816.lib.hardware;

import static org.junit.Assert.*;

import com.team1816.lib.hardware.YamlConfig.SubsystemConfig;
import java.io.InputStream;
import java.util.List;
import org.junit.Test;

public class YamlConfigTest {

    @Test
    public void subsystemConfig_merge() {
        InputStream baseConfigFile = getClass()
            .getClassLoader()
            .getResourceAsStream("test_base.config.yml");
        InputStream activeConfigFile = getClass()
            .getClassLoader()
            .getResourceAsStream("test_active.config.yml");

        SubsystemConfig base = YamlConfig
            .loadRaw(baseConfigFile)
            .subsystems.get("turret");
        SubsystemConfig active = YamlConfig
            .loadRaw(activeConfigFile)
            .subsystems.get("turret");
        SubsystemConfig result = SubsystemConfig.merge(active, base);
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
        assertTrue("implemented == true (favors true)", result.implemented);
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
        InputStream baseConfigFile = getClass()
            .getClassLoader()
            .getResourceAsStream("test_base.config.yml");
        InputStream activeConfigFile = getClass()
            .getClassLoader()
            .getResourceAsStream("test_active.config.yml");

        YamlConfig base = YamlConfig.loadRaw(baseConfigFile);
        YamlConfig active = YamlConfig.loadRaw(activeConfigFile);
        YamlConfig result = YamlConfig.merge(active, base);

        verifyMergedConfig(result);
    }

    @Test
    public void merged_implementedInherited_true() {
        SubsystemConfig base = new SubsystemConfig();
        base.implemented = true;

        SubsystemConfig active = new SubsystemConfig();
        active.implemented = null;

        SubsystemConfig merged = SubsystemConfig.merge(active, base);
        assertTrue("base == true", base.implemented);
        assertNull("active == null", active.implemented);
        assertTrue("merged == true", merged.implemented);
    }

    @Test
    public void merged_implementedInherited_false() {
        SubsystemConfig base = new SubsystemConfig();
        base.implemented = false;

        SubsystemConfig active = new SubsystemConfig();
        active.implemented = null;

        SubsystemConfig merged = SubsystemConfig.merge(active, base);
        assertFalse("base == false", base.implemented);
        assertNull("active == null", active.implemented);
        assertFalse("merged == false", merged.implemented);
    }

    @Test
    public void merged_implementedOverridden_true() {
        SubsystemConfig base = new SubsystemConfig();
        base.implemented = false;

        SubsystemConfig active = new SubsystemConfig();
        active.implemented = true;

        SubsystemConfig merged = SubsystemConfig.merge(active, base);
        assertFalse("base == false", base.implemented);
        assertTrue("active == true", active.implemented);
        assertTrue("merged == true", merged.implemented);
    }

    @Test
    public void merged_implementedOverridden_false() {
        SubsystemConfig base = new SubsystemConfig();
        base.implemented = true;

        SubsystemConfig active = new SubsystemConfig();
        active.implemented = false;

        SubsystemConfig merged = SubsystemConfig.merge(active, base);
        assertTrue("base == true", base.implemented);
        assertFalse("active == false", active.implemented);
        assertFalse("merged == false", merged.implemented);
    }

    @Test
    public void yamlConfig_autoMerge_ifExtends() throws ConfigIsAbstractException {
        var configFile = getClass()
            .getClassLoader()
            .getResourceAsStream("test_active.config.yml");
        YamlConfig config = YamlConfig.loadFrom(configFile);
        verifyMergedConfig(config);
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
