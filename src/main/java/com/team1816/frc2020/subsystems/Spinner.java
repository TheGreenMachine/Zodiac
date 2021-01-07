package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.revrobotics.ColorSensorV3;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class Spinner extends Subsystem {

    private static final String NAME = "spinner";
    private static Spinner INSTANCE;

    public static Spinner getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Spinner();
        }
        return INSTANCE;
    }

    // Components
    private final IMotorControllerEnhanced spinnerMotor;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    // State
    private double spinnerPow;
    private boolean outputsChanged = false;
    private boolean inPos = false;
    private SpinnerColor currentColor;
    private int colorsPassed = 0;
    private boolean colorMode = false;
    private boolean spinMode = false;

    // Constants
    public enum SpinnerColor {
        BLUE(138, 290, 627, 0),
        GREEN(201, 396, 350, 1),
        RED(702, 220, 155, 2),
        YELLOW(403, 390, 144, 3);

        public final int red;
        public final int green;
        public final int blue;
        public final int index;

        SpinnerColor(int red, int green, int blue, int index) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.index = index;
        }
    }

    private Spinner() {
        super(NAME);
        spinnerMotor = factory.getMotor(NAME, "spinner");
    }

    public void initialize() {
        currentColor = determineColor();
        colorsPassed = 0;
    }

    public void setSpinnerPower(double power) {
        spinnerPow = power;
        outputsChanged = true;
    }

    public void goToColor(Boolean held) {
        if (held) {
            if (!inPos) {
                System.out.println("Reached this level");
                spinnerPow = 0.05;
                outputsChanged = true;
                colorMode = true;
            } else {
                System.out.println("ENDED SPINNER SPIN");
                spinnerPow = 0.0;
                outputsChanged = true;
                colorMode = false;
            }
        } else {
            spinnerPow = 0.0;
            outputsChanged = true;
            colorMode = false;
        }
    }

    public void spinThreeTimes(boolean held) {
        if (held) {
            initialize();
            if (colorsPassed <= 28) {
                System.out.println("Reached this level");
                spinnerPow = 0.05;
                outputsChanged = true;
                spinMode = true;
            } else {
                System.out.println("ENDED SPINNER SPIN");
                spinnerPow = 0.0;
                outputsChanged = true;
                spinMode = false;
            }
        } else {
            spinnerPow = 0.0;
            outputsChanged = true;
            spinMode = false;
        }
    }

    public void stopSpin(boolean released) {
        if (released) {
            spinnerPow = 0.0;
            outputsChanged = false;
        }
    }

    @Override
    public void readPeriodicInputs() {
        SpinnerColor detectedColor = determineColor();
        //System.out.println("detected color index"detectedColor.index);
        //System.out.println("current color index"currentColor.index);

        //TODO: Fix NullPointerException here
        /*  if (detectedColor != currentColor) {
            if (detectedColor.index - currentColor.index == 1 || currentColor.index - detectedColor.index == 3) {
                colorsPassed++;
                currentColor = detectedColor;
            }
        } */
        //    System.out.println(colorsPassed);

        inPos = atColor(SpinnerColor.GREEN);
        if (colorMode && inPos) {
            spinnerPow = 0.0;
            outputsChanged = true;
            colorMode = false;
        }
        if (spinMode && colorsPassed >= 28) {
            spinnerPow = 0.0;
            outputsChanged = true;
            spinMode = false;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            spinnerMotor.set(ControlMode.PercentOutput, spinnerPow);
            outputsChanged = false;
        }
        //System.out.println("D: "+IR);
    }

    private SpinnerColor determineColor() {
        // TODO: Evaluate using built-in REV ColorMatch class?
        Color sensorColor = colorSensor.getColor();
        double redOut = sensorColor.red * 1247;
        double greenOut = sensorColor.green * 685;
        double blueOut = sensorColor.blue * 1354;

        SpinnerColor colorDetected = SpinnerColor.BLUE;
        double minError = Integer.MAX_VALUE;
        for (SpinnerColor color : SpinnerColor.values()) {
            double error =
                (Math.abs(redOut - color.red) / color.red) +
                (Math.abs(greenOut - color.green) / color.green) +
                (Math.abs(blueOut - color.blue) / color.blue);
            if (error < minError) {
                minError = error;
                colorDetected = color;
            }
        }

        //    System.out.println(colorDetected);
        return colorDetected;
    }

    public boolean atColor(SpinnerColor color) {
        return currentColor == color;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }
}
