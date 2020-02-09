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
    public void zenithYamlTest(){
        loadConfig("zenith");
    }

    private void loadConfig(String configName){
        var config = YamlConfig.loadFrom(this.getClass().getClassLoader().getResourceAsStream(configName + ".config.yml"));
        assertNotNull(config);
    }
}
