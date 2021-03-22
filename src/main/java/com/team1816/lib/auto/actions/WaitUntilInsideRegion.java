package com.team1816.lib.auto.actions;

import com.team1816.frc2020.RobotState;
import com.team254.lib.geometry.Translation2d;

public class WaitUntilInsideRegion implements Action {

    private static final RobotState mRobotState = RobotState.getInstance();

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;

    public WaitUntilInsideRegion(Translation2d bottomLeft, Translation2d topRight) {
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
    }

    @Override
    public boolean isFinished() {
        Translation2d position = mRobotState
            .getLatestFieldToVehicle()
            .getValue()
            .getTranslation();
        return (
            position.x() > mBottomLeft.x() &&
            position.x() < mTopRight.x() &&
            position.y() > mBottomLeft.y() &&
            position.y() < mTopRight.y()
        );
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {}
}
