package com.team1816.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.revrobotics.ColorSensorV3;
import com.team1816.frc2019.Robot;
import com.team1816.lib.hardware.RobotFactory;
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
    private SpinnerColor currentColor;
    private int colorsPassed = 0;

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
        RobotFactory factory = Robot.getFactory();
        spinnerMotor = factory.getMotor(NAME, "spinner");
    }

    public void initialize() {
        currentColor = determineColor();
    }

    public void setSpinnerPower(double power) {
        spinnerPow = power;
        outputsChanged = true;
    }

    @Override
    public void readPeriodicInputs() {
        SpinnerColor detectedColor = determineColor();
        if (detectedColor != currentColor) {
            if (detectedColor.index - currentColor.index == 1 || currentColor.index - detectedColor.index == 3) {
                colorsPassed++;
                currentColor = detectedColor;
            }

        }
        System.out.println(colorsPassed);
    }

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
                (Math.abs(redOut - color.red) / color.red)
                    + (Math.abs(greenOut - color.green) / color.green)
                    + (Math.abs(blueOut - color.blue) / color.blue);
            if (error < minError) {
                minError = error;
                colorDetected = color;
            }
        }

        System.out.println(colorDetected);
        System.out.print("R: " + sensorColor.red * 1.247);
        System.out.print("G: " + sensorColor.green * 0.685);
        System.out.println("B: " + sensorColor.blue * 1.354);
        return colorDetected;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}
