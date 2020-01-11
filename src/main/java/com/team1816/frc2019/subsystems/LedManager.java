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
    private boolean outputsChanged = true;

    private int ledR;
    private int ledG;
    private int ledB;

    private int ledBlinkR;
    private int ledBlinkG;
    private int ledBlinkB;

    private double period;
    private LedManager() {
        super(NAME);
        this.canifier = Robot.getFactory().getCanifier(NAME);
        this.ledR = 0;
        this.ledG = 0;
        this.ledB = 0;
    }
     public static LedManager getInstance(){
        if(INSTANCE==null){
            INSTANCE=new LedManager();
        }
        return INSTANCE;
    }

    public void setLedColor(int r, int g, int b) {
        if (this.ledR != r || this.ledG != g || this.ledB != b) {
            this.ledR = r;
            this.ledG = g;
            this.ledB = b;
            outputsChanged = true;
        }
    }

    public void setLedColorBlink(int r, int g, int b, double period) {
        blinkMode = true;
        this.ledBlinkR = r;
        this.ledBlinkG = g;
        this.ledBlinkB = b;
        this.period = period;
    }


    public void indicateStatus(RobotStatus status) {
        blinkMode = false;
        outputsChanged=true;
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void blinkStatus(RobotStatus status) {
        blinkMode = true;
        this.ledBlinkR = status.getRed();
        this.ledBlinkG = status.getGreen();
        this.ledBlinkB = status.getBlue();
    }


    public int[] getLedRgbBlink() {
        return new int[]{ledBlinkR, ledBlinkG, ledBlinkB};
    }

    public boolean getBlinkMode() {
        return blinkMode;
    }

    public double getPeriod(){return period;}

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged && canifier != null) {
            canifier.setLEDOutput((double) (ledG / 255.0), CANifier.LEDChannel.LEDChannelA);
            canifier.setLEDOutput((double) (ledR / 255.0), CANifier.LEDChannel.LEDChannelB);
            canifier.setLEDOutput((double) (ledB / 255.0), CANifier.LEDChannel.LEDChannelC);
            outputsChanged = false;
        }

        System.out.println("blink value: " + blinkMode);
        }


    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
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
    public void initSendable(SendableBuilder builder) { }

    public enum RobotStatus {
        ENABLED(223, 255, 0), // gross yellowgreen
        DISABLED(255, 103, 0), // orange
        ERROR(255, 0, 0), // red
        ENDGAME(0, 0, 255), // blue
        SEEN_TARGET(255, 0, 255), // magenta
        ON_TARGET(255, 0, 20), // deep magenta
        DRIVETRAIN_FLIPPED(0, 255, 0), // green
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
