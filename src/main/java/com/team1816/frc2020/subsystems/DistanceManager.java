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
        new Entry(1, 6100, 1, 1.5),//1.7),/* 167, 198 were 1.5 */
        new Entry(2, 10500, 1, 1.8),//1.7),
        new Entry(3, 8700, 1, 1.6),//1.6),
        new Entry(4, 8900, 0.5,1.65),
        new Entry(5, 9300, 0.5, 1.55),//1.5),

    };

    public double getShooterVelocity(double zone) {
        for (Entry bucket : buckets) {
            if (zone == bucket.distance) {
                System.out.println("Zone is"+zone);
                return bucket.shooterVelocity;
            }
        }
        return 9300;
    }

    public double getTurretBias(double zone) {
        for (Entry bucket : buckets) {
            if (zone <= bucket.distance) {
                return bucket.turretBias;
            }
        }
        return 1.1;
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
