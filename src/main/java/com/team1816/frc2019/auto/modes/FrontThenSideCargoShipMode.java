package com.team1816.frc2019.auto.modes;

import com.team1816.frc2019.RobotState;
import com.team1816.frc2019.paths.*;
import com.team1816.frc2019.subsystems.Drive;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class FrontThenSideCargoShipMode extends AutoModeBase {
    DrivePathAction first_path;
    DrivePathAction second_path;
    DrivePathAction third_path;
    DrivePathAction fourth_path;
    DrivePathAction fifth_path;
    boolean mLeft;
    boolean mStartHab1;

    public FrontThenSideCargoShipMode(boolean left, boolean startHab1) {
        mLeft = left;
        mStartHab1 = startHab1;
        if (mStartHab1) {
            first_path = new DrivePathAction(new Hab1ToCargoShipFrontPath(left));
        } else {
            first_path = new DrivePathAction(new GetOffHab2Path());
            second_path = new DrivePathAction(new Hab1ToCargoShipFrontPath(left));
        }
        third_path = new DrivePathAction(new CargoShipFrontToFeederPath(left));
        fourth_path = new DrivePathAction(new FeederToCargoShip1Path(left));
        fifth_path = new DrivePathAction(new CargoShip1ToCargoShip2Path(left));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        if (mStartHab1) {
            runAction(new ParallelAction(Arrays.asList(first_path)));
        } else {
            runAction(new ParallelAction(Arrays.asList(first_path)));
            runAction(new DriveOpenLoopAction(-0.15, -0.15, 0.75));
            runAction(new LambdaAction(() ->
                    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity())
            ));
            runAction(new LambdaAction(() -> Drive.getInstance().setHeading(Rotation2d.identity())));
            runAction(second_path);
        }

        runAction(new ParallelAction(Arrays.asList(third_path, new SeriesAction(Arrays.asList(
                new WaitForPathMarkerAction(CargoShipFrontToFeederPath.kLookForTargetMarker),
                new LambdaAction(() -> RobotState.getInstance().resetVision()))))));


        runAction(new ParallelAction(Arrays.asList(fifth_path, new SeriesAction(Arrays.asList(
                new WaitAction(0.25))))));
    }
}
