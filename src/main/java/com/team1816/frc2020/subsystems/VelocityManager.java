package com.team1816.frc2020.subsystems;

import java.util.List;
import java.util.Map;

public class VelocityManager {
    static class Entry {
        public final double shooterVelocity;
        public final double spindexerOutput;

        Entry(double shooterVelocity, double spindexerOutput) {
            this.shooterVelocity = shooterVelocity;
            this.spindexerOutput = spindexerOutput;
        }

        Entry() {
            this(0, 0);
        }
    }

    private final Map<Integer, Entry> velocities = Map.of(
        200, new Entry(Shooter.NEAR_VELOCITY, 1),
        300, new Entry(Shooter.MID_VELOCITY, 1),
        400, new Entry(Shooter.MID_FAR_VELOCITY, 1)
    );

    public double getShooterVelocity(double distance) {
        if (distance < 0) {
            return Shooter.NEAR_VELOCITY;
        } else if (distance < 250) {
            return velocities.get(200).shooterVelocity;
        } else if (distance < 350) {
            return velocities.get(300).shooterVelocity;
        } else if (distance < 450) {
            return velocities.get(400).shooterVelocity;
        } else {
            return Shooter.MAX_VELOCITY;
        }
    }

    public double getSpindexerOutput(double distance) {
        if (distance < 0) {
            return 1;
        } else if (distance < 250) {
            return velocities.get(200).spindexerOutput;
        } else if (distance < 350) {
            return velocities.get(300).spindexerOutput;
        } else if (distance < 450) {
            return velocities.get(400).spindexerOutput;
        } else {
            return 1;
        }
    }
}
