package com.team1816.lib.hardware;

import java.util.Objects;

public class SwerveModuleConfig {

    String drive;
    String azimuth;
    int encoderOffset;
    boolean invertSensorPhase = false;

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SwerveModuleConfig that = (SwerveModuleConfig) o;
        return (
            encoderOffset == that.encoderOffset &&
            Objects.equals(drive, that.drive) &&
            Objects.equals(azimuth, that.azimuth) &&
            invertSensorPhase == that.invertSensorPhase
        );
    }

    @Override
    public int hashCode() {
        return Objects.hash(drive, azimuth, encoderOffset, invertSensorPhase);
    }
}
