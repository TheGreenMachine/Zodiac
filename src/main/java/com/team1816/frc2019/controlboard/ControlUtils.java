package com.team1816.frc2019.controlboard;

import com.team254.lib.util.LatchedBoolean;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ControlUtils {
    public static PressAction createAction(BooleanSupplier input, Runnable action) {
        return new PressAction(input, action);
    }

    public static HoldAction createHoldAction(BooleanSupplier input, Consumer<Boolean> action) {
        return new HoldAction(input, action);
    }
//    public static HoldReleaseAction createHoldReleaseAction(BooleanSupplier input, Consumer<Boolean> actionHold,Consumer<Boolean> actionRelease) {
//        return new HoldReleaseAction(input, actionHold,actionRelease);
//    }

    public static ScalarAction createScalar(DoubleSupplier input, DoubleConsumer output) {
        return new ScalarAction(input, output);
    }

    public interface ButtonAction {
        void update();
    }

    public static class PressAction implements ButtonAction {
        private LatchedBoolean state = new LatchedBoolean();
        private BooleanSupplier input;
        private Runnable action;

        private PressAction(BooleanSupplier input, Runnable action) {
            this.input = input;
            this.action = action;
        }

        @Override
        public void update() {
            boolean inputPressed = input.getAsBoolean();
            boolean inputJustPressed = state.update(inputPressed);

            if (inputJustPressed) {
                action.run();
                state.update(false);
            }
        }


    }

    public static class HoldAction implements ButtonAction {
        private BooleanSupplier input;
        private Consumer<Boolean> action;
        private LatchedBoolean pressedState = new LatchedBoolean();
        private LatchedBoolean releasedState = new LatchedBoolean();

        private HoldAction(BooleanSupplier input, Consumer<Boolean> action) {
            this.input = input;
            this.action = action;
        }

        @Override
        public void update() {
            boolean inputPressed = input.getAsBoolean();
            boolean inputJustPressed = pressedState.update(inputPressed);
            boolean inputJustReleased = releasedState.update(!inputPressed);

            if (inputJustPressed) {
                action.accept(true);
            } else if (inputJustReleased) {
                action.accept(false);
            }
        }

    }
//    public static class HoldReleaseAction implements ButtonAction {
//        private BooleanSupplier input;
//        private Consumer<Boolean> holdAction;
//        private Consumer<Boolean> releaseAction;
//        private LatchedBoolean pressedState = new LatchedBoolean();
//        private LatchedBoolean releasedState = new LatchedBoolean();
//
//        private HoldReleaseAction(BooleanSupplier input, Consumer<Boolean> holdAction,Consumer<Boolean> releaseAction) {
//            this.input = input;
//            this.holdAction = holdAction;
//            this.releaseAction=releaseAction;
//        }
//
//        @Override
//        public void update() {
//            boolean inputPressed = input.getAsBoolean();
//            boolean inputJustPressed = pressedState.update(inputPressed);
//            boolean inputJustReleased = releasedState.update(!inputPressed);
//
//            if (inputJustPressed) {
//                holdAction.accept(true);
//            } else if (inputJustReleased) {
//                releaseAction.accept(true);
//            }
//        }
//
//    }

    public static class ScalarAction implements ButtonAction {
        private DoubleSupplier input;
        private DoubleConsumer action;

        private double lastValue;

        private ScalarAction(DoubleSupplier input, DoubleConsumer action) {
            this.input = input;
            this.action = action;
        }

        @Override
        public void update() {
            double newValue = input.getAsDouble();
            if (newValue != lastValue) {
                action.accept(newValue);
            }
            lastValue = newValue;
        }
    }
}
