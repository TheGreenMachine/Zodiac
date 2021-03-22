package com.team1816.lib.subsystems;

import com.team1816.frc2020.subsystems.Superstructure;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Subsystem to ensure the compressor never runs while the superstructure moves
 */
public class Infrastructure extends Subsystem {

    private static Infrastructure mInstance;

    private Superstructure mSuperstructure = Superstructure.getInstance();
    private ICompressor mCompressor;

    private boolean mIsManualControl = false;
    private static final boolean COMPRESSOR_ENABLED =
        factory.getConstant("compressorEnabled") > 0;
    private boolean lastCompressorOn = true;

    private Infrastructure() {
        super("Infrastructure");
        mCompressor = factory.getCompressor();
    }

    public static Infrastructure getInstance() {
        if (mInstance == null) {
            mInstance = new Infrastructure();
        }

        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Infrastructure.this) {
                        boolean superstructureMoving = !mSuperstructure.isAtDesiredState();

                        if (superstructureMoving || !mIsManualControl) {
                            if (lastCompressorOn) {
                                stopCompressor();
                                lastCompressorOn = false;
                            }
                        } else {
                            if (!lastCompressorOn) {
                                startCompressor();
                                lastCompressorOn = true;
                            }
                        }
                    }
                }

                @Override
                public void onStop(double timestamp) {}
            }
        );
    }

    public synchronized void setIsManualControl(boolean isManualControl) {
        mIsManualControl = isManualControl;

        if (mIsManualControl) {
            startCompressor();
        }
    }

    public synchronized boolean isManualControl() {
        return mIsManualControl;
    }

    private void startCompressor() {
        if (COMPRESSOR_ENABLED) {
            mCompressor.start();
        }
    }

    private void stopCompressor() {
        if (COMPRESSOR_ENABLED) {
            mCompressor.stop();
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
