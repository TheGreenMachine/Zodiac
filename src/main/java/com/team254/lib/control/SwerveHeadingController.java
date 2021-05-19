package com.team254.lib.control;

import com.team254.lib.util.SynchronousPIDF;
import edu.wpi.first.wpilibj.Timer;

public class SwerveHeadingController {
    private static SwerveHeadingController INSTANCE;
    private double mError;

    public static SwerveHeadingController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveHeadingController();
        }

        return INSTANCE;
    }

    private double targetHeading;
    private double disabledTimestamp;
    private double lastUpdateTimestamp;
    private final double disableTimeLength = 0.2;
    private SynchronousPIDF stabilizationPID;
    private SynchronousPIDF snapPID;
    private SynchronousPIDF stationaryPID;

    public enum State{
        Off, Stabilize, Snap, TemporaryDisable, Stationary
    }
    private State currentState = State.Off;
    public State getState(){
        return currentState;
    }
    private void setState(State newState){
        currentState = newState;
    }

    private SwerveHeadingController(){
        if (true){
            stabilizationPID = new SynchronousPIDF(0.005, 0.0, 0.0005, 0.0);
            snapPID = new SynchronousPIDF(0.0012, 0.00, 0.00, 0.0);
            stationaryPID = new SynchronousPIDF(0.01, 0.0, 0.002, 0.0);
        }else{
            stabilizationPID = new SynchronousPIDF(0.005, 0.0, 0.0005, 0.0);
            snapPID = new SynchronousPIDF(0.015, 0.0, 0.0, 0.0);
            stationaryPID = new SynchronousPIDF(0.01, 0.0, 0.002, 0.0);
        }

        targetHeading = 0;
        lastUpdateTimestamp = Timer.getFPGATimestamp();
    }

    public void setStabilizationTarget(double angle){
        targetHeading = angle;
        setState(State.Stabilize);
    }

    public void setSnapTarget(double angle){
        targetHeading = angle;
        setState(State.Snap);
    }

    public void setStationaryTarget(double angle){
        targetHeading = angle;
        setState(State.Stationary);
    }

    public void disable(){
        setState(State.Off);
    }

    public void temporarilyDisable(){
        setState(State.TemporaryDisable);
        disabledTimestamp = Timer.getFPGATimestamp();
    }

    public double getTargetHeading(){
        return targetHeading;
    }

    public double getError() {
        return mError;
    }

    public double updateRotationCorrection(double heading, double timestamp){
        double correction = 0;
        double error = heading - targetHeading;
        mError = error;
        double dt = timestamp - lastUpdateTimestamp;

        switch(currentState){
            case Off:

                break;
            case TemporaryDisable:
                targetHeading = heading;
                if(timestamp - disabledTimestamp >= disableTimeLength)
                    setState(State.Stabilize);
                break;
            case Stabilize:
                correction = stabilizationPID.calculate(error, dt);
                break;
            case Snap:
                correction = snapPID.calculate(error, dt);
                break;
            case Stationary:
                correction = stationaryPID.calculate(error, dt);
                break;
        }

        lastUpdateTimestamp = timestamp;
        return correction;
    }

}
