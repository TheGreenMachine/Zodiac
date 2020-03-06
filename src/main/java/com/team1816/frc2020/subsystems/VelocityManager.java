package com.team1816.frc2020.subsystems;

public class VelocityManager {
    private static VelocityManager INSTANCE;

    public static VelocityManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VelocityManager();
        }
        return INSTANCE;
    }

    private VelocityManager() {}
    static class Entry {
        public final double distance;
        public final double shooterVelocity;
        public final double spindexerOutput;

        Entry(double distance, double shooterVelocity, double spindexerOutput) {
            this.distance = distance;
            this.shooterVelocity = shooterVelocity;
            this.spindexerOutput = spindexerOutput;
        }

        Entry() {
            this(0, 0, 0);
        }
    }

    private final Entry[] velocities = new Entry[] {
        new Entry(167, 11_100, 2),
        new Entry(198, 10_400, 1),
        new Entry(223, 10_200, 1),
        new Entry(250, 9_900, 1),
        new Entry(285, 9_900, 1),
        new Entry(315, 10_200, 1),
        new Entry(360, 10_400, 1),
        new Entry(400, 11_100, 1)
    };

    public double getShooterVelocity(double distance) {
        for (Entry velocity : velocities) {
            if (distance <= velocity.distance) {
                return velocity.shooterVelocity;
            }
        }
        return Shooter.MAX_VELOCITY;
    }

    public double getSpindexerOutput(double distance) {
        for (Entry velocity : velocities) {
            if (distance <= velocity.distance) {
                return velocity.spindexerOutput;
            }
        }
        return 1;
    }
}
