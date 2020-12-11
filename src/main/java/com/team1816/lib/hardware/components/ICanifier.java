package com.team1816.lib.hardware.components;

import com.ctre.phoenix.CANifier;

public interface ICanifier {
    void setLedOutput(double percentOutput, CANifier.LEDChannel ledChannel);
}
