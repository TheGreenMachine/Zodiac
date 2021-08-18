package com.team1816.lib.subsystems;

import com.team1816.frc2020.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.loops.ILooper;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * initializing all member components at the start of the match.
 */
public abstract class Subsystem implements Sendable {

    private final String name;
    protected static final RobotFactory factory = Robot.getFactory();

    protected Subsystem(String name) {
        this.name = name;
        SendableRegistry.addLW(this, name, name);
    }

    public void writeToLog() {}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {}

    public void registerEnabledLoops(ILooper mEnabledLooper) {}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    @Deprecated
    public void outputTelemetry() {}

    @Override
    public void initSendable(SendableBuilder builder) {}

    public String getSubsystemName() {
        return name;
    }

    public boolean isImplemented() {
        return factory.getSubsystem(name).isImplemented();
    }
}
