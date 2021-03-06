package com.team1816.frc2020.subsystems;

public class DistanceManager {

    private static DistanceManager INSTANCE;

    public static DistanceManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new DistanceManager();
        }
        return INSTANCE;
    }

    private DistanceManager() {}

    static class Entry {

        public final double distance;
        public final double shooterVelocity;
        public final double spindexerOutput;
        public final double turretBias;

        Entry(
            double distance,
            double shooterVelocity,
            double spindexerOutput,
            double turretBias
        ) {
            this.distance = distance;
            this.shooterVelocity = shooterVelocity;
            this.spindexerOutput = spindexerOutput;
            this.turretBias = turretBias;
        }

        Entry() {
            this(0, 0, 0, 0);
        }
    }

    private final Entry[] buckets = new Entry[] {
        //        new Entry(167, 11_100, 2, 1.5), // untested
        //        new Entry(198, 10_400, 1, 1.5),
        //        new Entry(223, 8_300, 1, 1.45), //untested
        //        new Entry(250, 9_200, 1, 1.4),
        //        new Entry(285, 9_800, 1, 1.35),
        //        new Entry(315, 10_200, 1, 1.25),
        //        new Entry(360, 10_400, 1, 1.25),
        //        new Entry(400, 10_600, 1, 1.25)
        new Entry(167, 11_100, 2, 1.7),/* 167, 198 were 1.5 */
        new Entry(198, 10_400, 1, 1.7),
        new Entry(223, 9_200, 1, 1.6),
        new Entry(250, 9_200, 1, 1.5),
        new Entry(285, 10_000, 1, 1.8),
        new Entry(315, 10_100, 1, 1.875),
        new Entry(360, 10_400, 1, 1.95),
        new Entry(400, 10_600, 1, 2.1),
    };

    public double getShooterVelocity(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.shooterVelocity;
            }
        }
        return Shooter.MAX_VELOCITY;
    }

    public double getTurretBias(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.turretBias;
            }
        }
        return 1.25;
    }

    public double getSpindexerOutput(double distance) {
        for (Entry velocity : buckets) {
            if (distance <= velocity.distance) {
                return velocity.spindexerOutput;
            }
        }
        return 1;
    }
}
