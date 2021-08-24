package com.team1816.frc2020.controlboard;

import com.team1816.frc2020.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DemoModeControlBoard implements IControlBoard {
    private static DemoModeControlBoard INSTANCE = null;

    public static DemoModeControlBoard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new DemoModeControlBoard();
        }
        return INSTANCE;
    }

    private final Controller mController;
    private double drivetrainMultiplier = 0.2;

    private DemoModeControlBoard() {
        mController = new LogitechController(Constants.kDriveGamepadPort);
        SendableChooser<Double> speedChooser = new SendableChooser<>();
        speedChooser.setDefaultOption("Chill", 0.25);
        speedChooser.addOption("Standard", 0.5);
        speedChooser.addOption("Sport", 1.0);
        speedChooser.addOption("Park", 0.0);

        SmartDashboard.putData("DemoModeDriveSpeed", speedChooser);
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("DemoModeDriveSpeed")
            .addEntryListener(
                "selected",
                (table, key, entry, value, flags) -> {
                    switch (value.getString()) {
                        case "Chill":
                            drivetrainMultiplier = 0.2;
                            break;
                        case "Standard":
                            drivetrainMultiplier = 0.5;
                            break;
                        case "Sport":
                            drivetrainMultiplier = 1.0;
                            break;
                        default:
                            drivetrainMultiplier = 0;
                    }
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );


        drivetrainMultiplier = speedChooser.getSelected();
    }


    @Override
    public void reset() {

    }

    @Override
    public void setRumble(boolean on) {

    }

    @Override
    public boolean getSpinnerReset() {
        return false;
    }

    @Override
    public boolean getFeederFlapOut() {
        return false;
    }

    @Override
    public boolean getFeederFlapIn() {
        return false;
    }

    @Override
    public double getClimber() {
        return 0;
    }

    @Override
    public boolean getSpinnerColor() {
        return false;
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return false;
    }

    @Override
    public boolean getTurretJogLeft() {
        return mController.getDPad() == 270;
    }

    @Override
    public boolean getTurretJogRight() {
        return mController.getDPad() == 90;
    }

    @Override
    public boolean getShoot() {
        return mController.getTrigger(Controller.Side.RIGHT);
    }

    @Override
    public boolean getAutoAim() {
        return false;
    }

    @Override
    public boolean getCollectorBackSpin() {
        return false;
    }

    @Override
    public boolean getClimberDeploy() {
        return false;
    }

    @Override
    public boolean getFieldFollowing() {
        return mController.getDPad() == 180;
    }

    @Override
    public double getThrottle() {
        return drivetrainMultiplier * mController.getJoystick(
            Controller.Side.LEFT,
            Controller.Axis.Y
        );
    }

    @Override
    public double getTurn() {
        return drivetrainMultiplier * mController.getJoystick(
            Controller.Side.RIGHT,
            Controller.Axis.X
        );
    }

    @Override
    public double getStrafe() {
        return drivetrainMultiplier * mController.getJoystick(
            Controller.Side.LEFT,
            Controller.Axis.X
        );
    }

    @Override
    public boolean getQuickTurn() {
        return false;
    }

    @Override
    public boolean getSlowMode() {
        return false;
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return false;
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(LogitechController.Button.LB);
    }

    @Override
    public boolean getCollectorUp() {
        return false;
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return false;
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return false;
    }

    @Override
    public boolean getBrakeMode() {
        return false;
    }

    @Override
    public int getDriverClimber() {
        return 0;
    }

    @Override
    public double getDPad() {
        return 0;
    }

    @Override
    public boolean getFieldRelative() {
        return false;
    }

    @Override
    public boolean getHood() {
        return false;
    }
}
