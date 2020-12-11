package com.team1816.lib.hardware;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;

public class MotorUtil {
    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }

    public static void configCurrentLimit(
        IMotorControllerEnhanced motor,
        boolean enabled,
        double continuousCurrentLimit,
        double peakCurrentLimit,
        double peakThresholdSeconds
    ) {
        var limitConfig = new SupplyCurrentLimitConfiguration(enabled, continuousCurrentLimit, peakCurrentLimit, peakThresholdSeconds);
        if (motor instanceof TalonFX) {
            ((TalonFX) motor).configSupplyCurrentLimit(limitConfig);
        } else if (motor instanceof TalonSRX) {
            ((TalonSRX) motor).configSupplyCurrentLimit(limitConfig);
        }
    }
}
