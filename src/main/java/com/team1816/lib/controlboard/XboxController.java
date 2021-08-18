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
        mJoystickMap.put(Controller.Button.A,1);
        mJoystickMap.put(Controller.Button.B,2);
        mJoystickMap.put(Controller.Button.X,3);
        mJoystickMap.put(Controller.Button.Y,4);
        mJoystickMap.put(Controller.Button.LB,5);
        mJoystickMap.put(Controller.Button.RB,6);
        mJoystickMap.put(Controller.Button.BACK,7);
        mJoystickMap.put(Controller.Button.START,8);
        mJoystickMap.put(Controller.Button.L_JOYSTICK,9);
        mJoystickMap.put(Controller.Button.R_JOYSTICK,10);
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
