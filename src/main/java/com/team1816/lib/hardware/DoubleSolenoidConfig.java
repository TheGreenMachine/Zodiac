package com.team1816.lib.hardware;

import java.util.Objects;

public class DoubleSolenoidConfig {

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
