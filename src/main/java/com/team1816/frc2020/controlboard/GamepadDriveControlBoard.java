package com.team1816.frc2020.controlboard;

import com.team1816.frc2020.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import com.team1816.lib.controlboard.XboxController;
import com.team254.lib.geometry.Rotation2d;

import static com.team1816.frc2020.controlboard.ControlUtils.getControllerInstance;

public class GamepadDriveControlBoard implements IDriveControlBoard {

    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final Controller mController;

    private GamepadDriveControlBoard() {

        mController = getControllerInstance(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(
            LogitechController.Side.LEFT,
            LogitechController.Axis.Y
        );
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(
            LogitechController.Side.RIGHT,
            LogitechController.Axis.X
        );
    }

    @Override
    public double getStrafe() {
        return mController.getJoystick
            (
                XboxController.Side.LEFT,
                XboxController.Axis.X
            );
    }

    @Override
    public boolean getSlowMode() {
        return mController.getTrigger(LogitechController.Side.RIGHT);
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mController.getButton(LogitechController.Button.Y);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getButton(LogitechController.Button.R_JOYSTICK);
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(LogitechController.Button.LB);
    }

    @Override
    public boolean getCollectorUp() {
        return mController.getButton(LogitechController.Button.RB);
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mController.getButton(LogitechController.Button.X);
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return mController.getButton(LogitechController.Button.B);
    }

    @Override
    public boolean getBrakeMode() {
        return mController.getButton(LogitechController.Button.A);
    }

    @Override
    public boolean getFieldRelative() {
        return !mController.getButton(XboxController.Button.LB);
    }

    @Override
    public double getDPad() {
        if (mController.getDPad() == -1) {
            return -1;
        }

        if (mController.getTrigger(XboxController.Side.LEFT)) {
            double degs = SwerveCardinal.findClosest(Rotation2d.fromDegrees(mController.getDPad()), true).rotation.getDegrees();
            if (degs < 0) {
                degs += 360;
            }
            return degs;
        } else {
            double degs = SwerveCardinal.findClosest(Rotation2d.fromDegrees(mController.getDPad()), false).rotation.getDegrees();
            if (degs < 0) {
                degs += 360;
            }
            return degs;
        }
    }

    enum SwerveCardinal {
        BACK(180),
        FRONT(0),
        LEFT(90, -90, false),
        RIGHT(-90, 90, false),
        NONE(0),
        FRONT_LEFT(-30, 180, true),
        FRONT_RIGHT(-150, 90, true),
        BACK_LEFT(150, 0, true),
        BACK_RIGHT(30, -90, true);

        public final Rotation2d rotation;
        private final Rotation2d inputDirection;
        private final boolean isARocketCardinal;

        SwerveCardinal(double degrees) {
            this(degrees, degrees, false);
        }

        SwerveCardinal(double degrees, double inputDirectionDegrees, boolean isARocketCardinal) {
            rotation = Rotation2d.fromDegrees(degrees);
            inputDirection = Rotation2d.fromDegrees(inputDirectionDegrees);
            this.isARocketCardinal = isARocketCardinal;
        }

        public static SwerveCardinal findClosest(double xAxis, double yAxis, boolean isARocketCardinal) {
            return findClosest(new Rotation2d(yAxis, -xAxis, true), isARocketCardinal);
        }

        public static SwerveCardinal findClosest(Rotation2d stickDirection, boolean isARocketCardinal) {
            var values = SwerveCardinal.values();

            SwerveCardinal closest = null;
            double closestDistance = Double.MAX_VALUE;
            for (int i = 0; i < values.length; i++) {
                if (values[i].isARocketCardinal != isARocketCardinal) {
                    continue;
                }
                var checkDirection = values[i];
                var distance = Math.abs(stickDirection.distance(checkDirection.inputDirection));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = checkDirection;
                }
            }
            return closest;
        }

        public static boolean isDiagonal(SwerveCardinal cardinal) {
            return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
        }
    }

    @Override
    public int getDriverClimber() {
        switch (mController.getDPad()) {
            case 0:
            case 45:
            case 315:
                return 1;
            case 180:
            case 135:
            case 225:
                return -1;
            default:
                return 0;
        }
    }
}
