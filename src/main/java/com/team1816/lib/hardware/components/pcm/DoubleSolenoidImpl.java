package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class DoubleSolenoidImpl extends DoubleSolenoid implements IDoubleSolenoid {

    public DoubleSolenoidImpl(int forwardChannel, int reverseChannel) {
        super(forwardChannel, reverseChannel);
    }

    public DoubleSolenoidImpl(int moduleNumber, int forwardChannel, int reverseChannel) {
        super(moduleNumber, forwardChannel, reverseChannel);
    }
}
