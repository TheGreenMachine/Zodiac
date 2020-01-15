package com.team1816.frc2019.subsystems;

import com.ctre.phoenix.CANifier;
import com.team1816.frc2019.Robot;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class LedManager extends Subsystem {
    public static final String NAME = "ledmanager";

    private static LedManager INSTANCE;

    private CANifier canifier;

    private boolean blinkMode;
    private boolean blinkLedOn = false;
    private boolean outputsChanged = true;

    private int ledR;
    private int ledG;
    private int ledB;

    private int period; // ms
    private long lastWriteTime = System.currentTimeMillis();

    private LedManager() {
        super(NAME);
        this.canifier = Robot.getFactory().getCanifier(NAME);
        this.ledR = 0;
        this.ledG = 0;
        this.ledB = 0;
    }

    public static LedManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LedManager();
        }
        return INSTANCE;
    }

    @Deprecated
    public void forceSetLedColor(int r, int g, int b) {
        if (this.ledR != r || this.ledG != g || this.ledB != b) {
            canifier.setLEDOutput((ledG / 255.0), CANifier.LEDChannel.LEDChannelA);
            canifier.setLEDOutput((ledR / 255.0), CANifier.LEDChannel.LEDChannelB);
            canifier.setLEDOutput((ledB / 255.0), CANifier.LEDChannel.LEDChannelC);
        }
    }

    public void setLedColor(int r, int g, int b) {
        if (this.ledR != r || this.ledG != g || this.ledB != b) {
            this.ledR = r;
            this.ledG = g;
            this.ledB = b;
            outputsChanged = true;
        }
    }

    public void setLedColorBlink(int r, int g, int b, int period) {
        // Period is in milliseconds
        setLedColor(r, g, b);
        blinkMode = true;
        this.period = period;
        outputsChanged = true;
    }

    public void setLedColorBlink(int r, int g, int b) {
        // Default period of 1 second
        setLedColorBlink(r, g, b, 1000);
    }

    public void indicateStatus(RobotStatus status) {
        blinkMode = false;
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void blinkStatus(RobotStatus status) {
        setLedColorBlink(status.getRed(), status.getGreen(), status.getBlue());
    }


    public int[] getLedColor() {
        return new int[] { ledR, ledG, ledB };
    }

    public boolean isBlinkMode() {
        return blinkMode;
    }

    public double getPeriod() {
        return period;
    }

    private void writeLedHardware(int r, int g, int b) {
        canifier.setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
    }

    @Override
    public void writePeriodicOutputs() {
        if (blinkMode) {
            if (System.currentTimeMillis() >= lastWriteTime + period) {
                if (blinkLedOn) {
                    writeLedHardware(0, 0, 0);
                    blinkLedOn = false;
                } else {
                    writeLedHardware(ledR, ledG, ledB);
                    blinkLedOn = true;
                }
            }
        } else if (outputsChanged && canifier != null) {
            writeLedHardware(ledR, ledG, ledB);
            outputsChanged = false;
        }

        System.out.println("blink value: " + blinkMode);
    }


    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        // TODO: Refactor checkSystem() to not directly call writePeriodicOutputs()
        System.out.println("Warning: checking LED systems");
        Timer.delay(3);

        setLedColor(255, 0, 0);
        writePeriodicOutputs();
        Timer.delay(0.4);
        setLedColor(0, 255, 0);
        writePeriodicOutputs();
        Timer.delay(0.4);
        setLedColor(0, 0, 255);
        writePeriodicOutputs();
        Timer.delay(0.4);
        setLedColor(0, 0, 0);
        writePeriodicOutputs();

        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    public enum RobotStatus {
        ENABLED(0, 255, 0), // green
        DISABLED(255, 103, 0), // orange
        ERROR(255, 0, 0), // red
        AUTONOMOUS(0, 255, 255), // cyan (we can also try 42, 161, 152)
        ENDGAME(0, 0, 255), // blue
        SEEN_TARGET(255, 0, 255), // magenta
        ON_TARGET(255, 0, 20), // deep magenta
        DRIVETRAIN_FLIPPED(223, 255, 0), // gross yellowgreen
        OFF(0, 0, 0); // off

        int red;
        int green;
        int blue;

        RobotStatus(int r, int g, int b) {
            this.red = r;
            this.green = g;
            this.blue = b;
        }

        public int getRed() {
            return red;
        }

        public int getGreen() {
            return green;
        }

        public int getBlue() {
            return blue;
        }
    }
}
