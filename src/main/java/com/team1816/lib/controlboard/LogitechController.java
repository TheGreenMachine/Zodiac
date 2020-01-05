package com.team1816.lib.controlboard;

public class LogitechController extends Controller {

    @Override
    protected int getLeftAxisId() {
        return 0;
    }

    @Override
    protected int getRightAxisId() {
        return 2;
    }

    public LogitechController(int port) {
        super(port);
    }

    public enum Button implements Controller.Button {
        A(2), B(3), X(1), Y(4), LB(5), RB(6), BACK(9), START(10), L_JOYSTICK(11), R_JOYSTICK(12);

        public final int id;

        Button(int id) {
            this.id = id;
        }

        @Override
        public int getId() {
            return this.id;
        }
    }

    @Override
    public boolean getTrigger(Side side) {
        return mController.getRawButton(side == Side.LEFT ? 7 : 8);
    }

    @Override
    public double getTriggerScalar(Controller.Side side) {
        return getTrigger(side) ? 1 : 0;
    }
}
