package com.team1816.lib.hardware;

import org.junit.Test;

import static org.junit.Assert.assertNotNull;

public class RobotYamlTests {

    @Test
    public void defaultYamlTest(){
        loadConfig("default");
    }

    @Test
    public void cheezeCurdYamlTest(){
        loadConfig("CheezeCurd");
    }

    @Test
    public void zetaYamlTest(){
        loadConfig("zeta");
    }

    @Test
    public void zodiacYamlTest(){
        loadConfig("zodiac");
    }

    @Test
    public void zodiacProYamlTest() { loadConfig("zodiac_pro"); }

    @Test
    public void zenithYamlTest(){
        loadConfig("zenith");
    }

    private void loadConfig(String configName) {
        YamlConfig config = null;
        try {
            config = YamlConfig.loadFrom(this.getClass().getClassLoader().getResourceAsStream(configName + ".config.yml"));
        } catch (ConfigIsAbstractException e) {
            e.printStackTrace();
        }
        assertNotNull(config);
        System.out.println(config);
    }
}
