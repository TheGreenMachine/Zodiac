package com.team1816.lib.hardware;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.frc2020.Constants;
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

    /**
     * @deprecated Use {@link IMotorControllerEnhanced#configSupplyCurrentLimit(SupplyCurrentLimitConfiguration, int)} instead.
     */
    @Deprecated
    public static void configCurrentLimit(
        IMotorControllerEnhanced motor,
        boolean enabled,
        double continuousCurrentLimit,
        double peakCurrentLimit,
        double peakThresholdSeconds
    ) {
        var limitConfig = new SupplyCurrentLimitConfiguration(
            enabled,
            continuousCurrentLimit,
            peakCurrentLimit,
            peakThresholdSeconds
        );
        motor.configSupplyCurrentLimit(limitConfig, Constants.kCANTimeoutMs);
    }

    public static double getSupplyCurrent(IMotorControllerEnhanced motor) {
        // If only CTRE had these methods in the interface...
        if (motor instanceof TalonFX) {
            return ((TalonFX) motor).getSupplyCurrent();
        } else if (motor instanceof TalonSRX) {
            return ((TalonSRX) motor).getSupplyCurrent();
        }
        return 0;
    }
}
