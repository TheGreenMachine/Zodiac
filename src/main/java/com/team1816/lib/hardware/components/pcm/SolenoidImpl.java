package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidImpl extends Solenoid implements ISolenoid {

    public SolenoidImpl(int channel) {
        super(channel);
    }

    public SolenoidImpl(int moduleNumber, int channel) {
        super(moduleNumber, channel);
    }
}
