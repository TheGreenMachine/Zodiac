package com.team1816.frc2020.controlboard;

import com.team1816.frc2020.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;

public class DemoModeControlBoard implements IControlBoard {
    private static DemoModeControlBoard INSTANCE = null;

    public static DemoModeControlBoard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new DemoModeControlBoard();
        }
        return INSTANCE;
    }

    private final Controller mController;
    private final Map<String, Double> driveModeMap = Map.of(
        DEFAULT_DRIVE_MODE, 0.25,
        "Standard", 0.5,
        "Sport", 1.0,
        "Park", 0.0
    );
    private double drivetrainMultiplier;

    // Constants
    private static final String DEFAULT_DRIVE_MODE = "Chill";;

    private DemoModeControlBoard() {
        mController = new LogitechController(Constants.kDriveGamepadPort);
        SendableChooser<Void> speedChooser = new SendableChooser<>();
        for (String mode : driveModeMap.keySet()) {
            if (mode.equals(DEFAULT_DRIVE_MODE)) {
                speedChooser.setDefaultOption(mode, null);
            }
            speedChooser.addOption(mode, null);
        }

        drivetrainMultiplier = driveModeMap.get(DEFAULT_DRIVE_MODE);
        SmartDashboard.putNumber("DrivetrainMultiplier", drivetrainMultiplier);

        SmartDashboard.putData("DemoModeDriveSpeed", speedChooser);
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("DemoModeDriveSpeed")
            .addEntryListener(
                "selected",
                (table, key, entry, value, flags) -> {
                    // This is all because SendableChooser.getSelected() does not update when listener is fired
                    drivetrainMultiplier = driveModeMap.get(value.getString());
                    SmartDashboard.putNumber("DrivetrainMultiplier", drivetrainMultiplier);
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );
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
    public boolean getClimberUp() {
        return false;
    }

    @Override
    public boolean getClimberDown(){return false;}

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
