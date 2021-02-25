package com.team1816.lib.hardware;

import java.util.Objects;

public class SwerveModuleConfig {
    String drive;
    String azimuth;

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SwerveModuleConfig that = (SwerveModuleConfig) o;
        return Objects.equals(drive, that.drive) && Objects.equals(azimuth, that.azimuth);
    }

    @Override
    public int hashCode() {
        return Objects.hash(drive, azimuth);
    }

    @Override
    public String toString() {
        return "SwerveModuleConfig{" +
            "drive='" + drive + '\'' +
            ", azimuth='" + azimuth + '\'' +
            '}';
    }
}
