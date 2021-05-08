package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.Compressor;

public class CompressorImpl extends Compressor implements ICompressor {

    public CompressorImpl(int module) {
        super(module);
    }

    public CompressorImpl() {
        super();
    }
}
