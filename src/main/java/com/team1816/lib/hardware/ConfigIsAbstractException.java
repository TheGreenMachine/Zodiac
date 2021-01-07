package com.team1816.lib.hardware;

public class ConfigIsAbstractException extends Exception {

    public ConfigIsAbstractException() {
        super("Cannot instantiate config marked as abstract!");
    }
}
