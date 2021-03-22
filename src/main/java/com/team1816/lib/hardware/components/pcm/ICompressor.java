package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.Sendable;

public interface ICompressor extends Sendable, AutoCloseable {
    void start();

    void stop();

    boolean enabled();

    double getCompressorCurrent();
}
