package com.team1816.lib.controlboard;

public class WasdController extends Controller{

    public WasdController(int port) {
        super(port);
        mJoystickMap.put(Controller.Button.A,2);
        mJoystickMap.put(Controller.Button.B,3);
        mJoystickMap.put(Controller.Button.X,1);
        mJoystickMap.put(Controller.Button.Y,4);
        mJoystickMap.put(Controller.Button.LB,5);
        mJoystickMap.put(Controller.Button.RB,6);
        mJoystickMap.put(Controller.Button.BACK,9);
        mJoystickMap.put(Controller.Button.START,10);
        mJoystickMap.put(Controller.Button.L_JOYSTICK,11);
        mJoystickMap.put(Controller.Button.R_JOYSTICK,12);
    }

    @Override
    protected int getLeftAxisId() {
        return 1;
    }

    @Override
    protected int getRightAxisId() {
        return 0;
    }

    @Override
    public double getTriggerScalar(Side side) {
        return 2;
    }
}
