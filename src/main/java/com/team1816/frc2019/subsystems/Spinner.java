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
    private static final String NAME="spinner";
    private static Spinner INSTANCE;

    public static Spinner getINSTANCE(){
        if(INSTANCE==null){
            INSTANCE=new Spinner();
        }
        return INSTANCE;
    }

    private final IMotorControllerEnhanced spinnerMotor;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private double spinnerPow;
    private boolean outputsChanged=false;
    //                           R    G   B
    private final double[] Blue={138,290,627};
    private final double[] Green={201,396,350};
    private final double[] Red={702,220,155};
    private final double[] Yellow={403,390,144};
    private final double[][]colors={Blue,Green,Red,Yellow};
    int color=0;
    int colorsPassed=0;
    boolean colorChanged=false;


    private final RobotFactory factory=Robot.getFactory();

    private Spinner() {
        super(NAME);
        spinnerMotor=factory.getMotor(NAME,"spinner");
    }

    public void initialize(){
        color=detetermineColor(m_colorSensor,colors);
    }

    public void setSpinnerPower(double power){
        spinnerPow=power;
        outputsChanged=true;
    }

    public void writePeriodicOutputs() {
        if (outputsChanged) {
            spinnerMotor.set(ControlMode.PercentOutput, spinnerPow);
            outputsChanged = false;
        }
        int detectedColor=detetermineColor(m_colorSensor,colors);
        if(detectedColor!=color){
            if(detectedColor-color==1||color-detectedColor==3){
                colorsPassed++;
                color=detectedColor;
            }

        }
        System.out.println(colorsPassed);

        //System.out.println("D: "+IR);
    }

    private static int detetermineColor(ColorSensorV3 m_colorSensor,double[][]colors){
        Color detectedColor = m_colorSensor.getColor();
        double IR = m_colorSensor.getIR();
        double redOut=detectedColor.red*1247;
        double greenOut=detectedColor.green*685;
        double blueOut=detectedColor.blue*1354;
        int colorDetected=0;
        double error= Integer.MAX_VALUE;
        for(int i=0;i<4;i++){
            double[] color=colors[i];
            double errorTemp=Math.abs(redOut-color[0])/color[0]+Math.abs(greenOut-color[1])/color[1]+Math.abs(blueOut-color[2])/color[2];
            if(error>errorTemp){
                error=errorTemp;
                colorDetected=i;
            }
        }
        if(colorDetected==0){
            System.out.println("Blue");
        }
        else if(colorDetected==1){
            System.out.println("Green");
        }
        else if(colorDetected==2){
            System.out.println("Red");
        }
        else if(colorDetected==3){
            System.out.println("Yellow");
        }
        System.out.print("R: "+detectedColor.red*1.247);
        System.out.print("G: "+detectedColor.green*0.685);
        System.out.println("B: "+detectedColor.blue*1.354);
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
