package com.team1816.lib.hardware;

import java.util.Objects;

public class PidConfig {

    public static final PidConfig EMPTY = new PidConfig(0, 0, 0, 0);

    double kP;
    double kI;
    double kD;
    double kF;

    public PidConfig() {}

    public PidConfig(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public double getkF() {
        return kF;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PidConfig pidConfig = (PidConfig) o;
        return (
            Double.compare(pidConfig.kP, kP) == 0 &&
            Double.compare(pidConfig.kI, kI) == 0 &&
            Double.compare(pidConfig.kD, kD) == 0 &&
            Double.compare(pidConfig.kF, kF) == 0
        );
    }

    @Override
    public int hashCode() {
        return Objects.hash(kP, kI, kD, kF);
    }

    @Override
    public String toString() {
        return "kP = " + kP +
            ", kI = " + kI +
            ", kD = " + kD +
            ", kF = " + kF;
    }
}
