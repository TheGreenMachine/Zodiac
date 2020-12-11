package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class GhostDoubleSolenoid implements IDoubleSolenoid {
    // State
    private DoubleSolenoid.Value value;

    @Override
    public DoubleSolenoid.Value get() {
        return value;
    }

    @Override
    public void set(DoubleSolenoid.Value value) {
        this.value = value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Double Solenoid");
        builder.setActuator(true);
        builder.setSafeState(() -> set(DoubleSolenoid.Value.kOff));
        builder.addStringProperty("Value", () -> get().name().substring(1), value -> {
            if ("Forward".equals(value)) {
                set(DoubleSolenoid.Value.kForward);
            } else if ("Reverse".equals(value)) {
                set(DoubleSolenoid.Value.kReverse);
            } else {
                set(DoubleSolenoid.Value.kOff);
            }
        });
    }

    @Override
    public void close() throws Exception {

    }
}
