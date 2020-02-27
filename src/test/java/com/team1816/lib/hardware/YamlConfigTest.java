package com.team1816.lib.hardware;

import org.junit.Test;

import java.io.InputStream;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class YamlConfigTest {

    @Test
    public void subsystemConfig_merge() {
        InputStream baseConfigFile = getClass().getClassLoader().getResourceAsStream("test_base.config.yml");
        InputStream activeConfigFile = getClass().getClassLoader().getResourceAsStream("test_active.config.yml");

        YamlConfig.SubsystemConfig base = YamlConfig.loadRaw(baseConfigFile).subsystems.get("turret");
        YamlConfig.SubsystemConfig active = YamlConfig.loadRaw(activeConfigFile).subsystems.get("turret");
        YamlConfig.SubsystemConfig result = YamlConfig.SubsystemConfig.merge(active, base);
        System.out.println(result);

        assertEquals("Base constant kP == 2.83", 2.83, result.constants.get("kP"), 0);
        assertEquals("Overridden constant minPos == -374", -374, result.constants.get("minPos").intValue());
        assertEquals("New constant newConstant == 34", 34, result.constants.get("newConstant"), 0);
        assertEquals("Turret talon should be overridden to 13", 13, result.talons.get("turret").intValue());
        assertTrue("implemented == true (favors true)", result.implemented);
    }

    @Test(expected = ConfigIsAbstractException.class)
    public void loadFromBase_throwsIfAbstract() throws ConfigIsAbstractException {
        YamlConfig.loadFrom(getClass().getClassLoader().getResourceAsStream("test_base.config.yml"));
    }

    @Test
    public void loadFromActive_doesNotThrow() throws ConfigIsAbstractException {
        YamlConfig.loadFrom(getClass().getClassLoader().getResourceAsStream("test_active.config.yml"));
    }

}
