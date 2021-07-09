package com.team1816.lib.subsystems;

import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.loops.Looper;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

    public static SubsystemManager mInstance = null;

    private List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    private boolean asyncInitialized = false;

    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }

        return mInstance;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem s : mAllSubsystems) {
            System.out.println("SUBSYSTEM: " + s.getName());
            ret_val &= s.checkSystem();
        }

        return ret_val;
    }

    public void initAsync() {
        try {
            for (Subsystem subsystem : mAllSubsystems) {
                subsystem.initAsync().get();
            }
            asyncInitialized = true;
        } catch (InterruptedException | ExecutionException e) {
            DriverStation.reportError("Error initializing async subsystems", e.getStackTrace());
            asyncInitialized = false;
        }
    }

    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }

    public void setSubsystems(Subsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
        for (Subsystem subsystem : mAllSubsystems) {
            if (!subsystem.isImplemented()) {
                System.out.println(
                    "  Warning: " + subsystem.getName() + " is not implemented"
                );
            }
        }
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            mLoops.forEach(l -> l.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            if (!asyncInitialized) {
                DriverStation.reportError("Subsystem Async Initializers not called! Skipping loop", false);
                return;
            }
            mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
            mLoops.forEach(l -> l.onLoop(timestamp));
            mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
        }

        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}
