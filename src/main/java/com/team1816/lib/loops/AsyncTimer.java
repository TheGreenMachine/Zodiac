package com.team1816.lib.loops;

import edu.wpi.first.wpilibj.Timer;

import javax.annotation.Nullable;

public class AsyncTimer {
    private long startTime;
    private int duration;
    private Runnable startAction;
    private Runnable endAction;
    private boolean hasStarted;
    private boolean completed;

    public AsyncTimer(int duration, @Nullable Runnable startAction, Runnable endAction) {
        this.duration = duration;
        this.startAction = startAction;
        this.endAction = endAction;
        this.hasStarted = false;
        this.completed = false;
    }

    public AsyncTimer(int duration, Runnable endAction) {
        this(duration, null, endAction);
    }

    public void update() {
        if (completed) return;

        if (!hasStarted) {
            startTime = (long) (Timer.getFPGATimestamp() * 1000);
            if (startAction != null) {
                startAction.run();
            }
        } else {
            if (Timer.getFPGATimestamp() * 1000 >= startTime + duration) {
                completed = true;
                endAction.run();
            }
        }
    }

    public boolean isCompleted() {
        return completed;
    }

    public void reset() {
        completed = false;
        hasStarted = false;
        startTime = 0;
    }
}
