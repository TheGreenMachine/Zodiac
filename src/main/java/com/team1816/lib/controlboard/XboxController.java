package com.team1816.lib.controlboard;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class XboxController extends Controller {

    @Override
    protected int getLeftAxisId() {
        return 0;
    }

    @Override
    protected int getRightAxisId() {
        return 4;
    }

    public XboxController(int port) {
        super(port);
    }

    public enum Button implements Controller.Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

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
    public double getTriggerScalar(Controller.Side side) {
        return mController.getRawAxis(side == Side.LEFT ? 2 : 3);
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(RumbleType.kRightRumble, on ? 1 : 0);
    }

}
