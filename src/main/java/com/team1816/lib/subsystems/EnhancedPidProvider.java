package com.team1816.lib.subsystems;

import com.team1816.lib.hardware.PidConfig;

public interface EnhancedPidProvider extends PidProvider {
    PidConfig getPidConfig();

    default double getKP() {
        return getPidConfig().getkP();
    }
    default double getKI() {
        return getPidConfig().getkI();
    }

    default double getKD() {
        return getPidConfig().getkD();
    }

    default double getKF() {
        return getPidConfig().getkF();
    }

    default String pidToString() {
        return getPidConfig().toString();
    }
}
