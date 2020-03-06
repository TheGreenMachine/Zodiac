package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX extends TalonFX implements ILazyMotorControllerEnhanced  {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            System.out.println("Writing output value to can ID " + getDeviceID() +
                " ControlMode: " + mode +
                " value: " + value);
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }
}
