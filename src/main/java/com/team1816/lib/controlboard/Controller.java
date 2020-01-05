package com.team1816.lib.controlboard;

import com.team1816.frc2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public abstract class Controller {
    protected final Joystick mController;

    public enum Side {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    public interface Button {
        int getId();
    }

    public double getJoystick(Controller.Side side, Controller.Axis axis) {
        double deadband = Constants.kJoystickThreshold;

        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        // multiplies by -1 if y-axis (inverted normally)
        return handleDeadband(
            (y ? -1 : 1)
                * mController.getRawAxis(
                    (left ? getLeftAxisId() : getRightAxisId()) + (y ? 1 : 0)
                ),
            deadband
        );
    }

    protected abstract int getLeftAxisId();
    protected abstract int getRightAxisId();

    public Controller(int port) {
        mController = new Joystick(port);
    }

    public abstract double getTriggerScalar(Side side);

    public void setRumble(boolean on) { /* no-op */ }

    public int getDPad() {
        return mController.getPOV();
    }

    public boolean getButton(Button button) {
        return mController.getRawButton(button.getId());
    }

    public boolean getTrigger(Side side) {
        return getTriggerScalar(side) > Constants.kJoystickThreshold;
    }

    protected double handleDeadband(double value, double deadband) {
        return (Math.abs(value) > Math.abs(deadband)) ? value : 0;
    }
}
