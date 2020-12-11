package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frc2020.Robot;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

@Deprecated
public class TalonSRXChecker {
    public static class CheckerConfig {
        public double mCurrentFloor = 5;
        public double mRPMFloor = 2000;

        public double mCurrentEpsilon = 5.0;
        public double mRPMEpsilon = 500;
        public DoubleSupplier mRPMSupplier = null;

        public double mRunTimeSec = 4.0;
        public double mWaitTimeSec = 2.0;
        public double mRunOutputPercentage = 0.2;

        public static CheckerConfig getForSubsystemMotor(Subsystem subsystem, IMotorControllerEnhanced motor) {
            var name = subsystem.getName();
            var factory = Robot.getFactory();
            return new TalonSRXChecker.CheckerConfig() {
                {
                    mCurrentFloor = factory.getConstant(name,"currentFloorCheck");
                    mRPMFloor = factory.getConstant(name,"rpmFloorCheck");
                    mCurrentEpsilon = factory.getConstant(name,"currentEpsilonCheck");
                    mRPMEpsilon = factory.getConstant(name,"rpmEpsilonCheck");
                    mRPMSupplier = () -> motor.getSelectedSensorVelocity(0);
                }
            };
        }
    }

    public static class TalonSRXConfig {
        public String mName;
        public IMotorControllerEnhanced mTalon;

        public TalonSRXConfig(String name, IMotorControllerEnhanced talon) {
            mName = name;
            mTalon = talon;
        }
    }

    private static class StoredTalonSRXConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    public static boolean checkMotors(Subsystem subsystem, ArrayList<TalonSRXConfig> talonsToCheck,
                                      CheckerConfig checkerConfig) {
        boolean failure = false;
        System.out.println("////////////////////////////////////////////////");
        System.out.println("Checking subsystem " + subsystem.getClass() + " for " + talonsToCheck.size() + " talons.");

        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();
        ArrayList<StoredTalonSRXConfiguration> storedConfigurations = new ArrayList<>();

        // Record previous configuration for all talons.
        for (TalonSRXConfig config : talonsToCheck) {
            if (config.mTalon.getDeviceID() < 0)
                continue;
            IMotorControllerEnhanced talon = config.mTalon;

            StoredTalonSRXConfiguration configuration = new StoredTalonSRXConfiguration();
            configuration.mMode = talon.getControlMode();
            storedConfigurations.add(configuration);

            // Now set to disabled.
            talon.set(ControlMode.PercentOutput, 0.0);
        }

        for (TalonSRXConfig config : talonsToCheck) {
            System.out.println("Checking: " + config.mName);

            if (config.mTalon.getDeviceID() < 0) {
                System.out.println("Talon Disabled Checks Skipped!!");
                continue;
            }

            config.mTalon.set(ControlMode.PercentOutput, checkerConfig.mRunOutputPercentage);
            Timer.delay(checkerConfig.mRunTimeSec);

            // Now poll the interesting information.
            double current = config.mTalon.getOutputCurrent();
            currents.add(current);
            System.out.print("Current: " + current);

            double rpm = Double.NaN;
            if (checkerConfig.mRPMSupplier != null) {
                rpm = checkerConfig.mRPMSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.print(" RPM: " + rpm);
            }
            System.out.print('\n');

            config.mTalon.set(ControlMode.PercentOutput, 0.0);

            // And perform checks.
            if (current < checkerConfig.mCurrentFloor) {
                DriverStation.reportError(
                    config.mName + " has failed current floor check vs " + checkerConfig.mCurrentFloor + "!!!!!!!!!!!!", false);
                failure = true;
            }
            if (checkerConfig.mRPMSupplier != null) {
                if (rpm < checkerConfig.mRPMFloor) {
                    DriverStation.reportError(
                        config.mName + " has failed rpm floor check vs " + checkerConfig.mRPMFloor + "!!!!!!!!!!!!!", false);
                    failure = true;
                }
            }

            Timer.delay(checkerConfig.mWaitTimeSec);
        }

        // Now run aggregate checks.

        if (currents.size() > 0) {
            Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(currents, average, checkerConfig.mCurrentEpsilon)) {
                DriverStation.reportError("Currents varied!!!!!!!!!!!", false);
                failure = true;
            }
        }

        if (rpms.size() > 0) {
            Double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(rpms, average, checkerConfig.mRPMEpsilon)) {
                DriverStation.reportError("RPMs varied!!!!!!!!", false);
                failure = true;
            }
        }

        // Restore Talon configurations
        for (int i = 0; i < talonsToCheck.size(); ++i) {
            IMotorControllerEnhanced talon = talonsToCheck.get(i).mTalon;
            if (talon.getDeviceID() >= 0) {
                talon.set(storedConfigurations.get(i).mMode, 0);
            }
        }

        return !failure;
    }
}
