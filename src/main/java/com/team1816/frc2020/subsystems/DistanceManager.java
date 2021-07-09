package com.team1816.frc2020.subsystems;

import com.team1816.frc2020.Robot;

public class DistanceManager {

    private static DistanceManager INSTANCE;

    public static DistanceManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new DistanceManager();
        }
        return INSTANCE;
    }

    // Components
    private final Camera camera = Camera.getInstance();

    // State
    private final Entry[] buckets;
    private int zone;

    // Constants
    public static final boolean USE_ZONES = Robot.getFactory().getConstant("useZones") > 0;

    private DistanceManager() {
        if (USE_ZONES) {
            buckets = zone_buckets;
        } else {
            buckets = distance_buckets;
        }
    }

    static class Entry {

        public final double distance;
        public final double shooterVelocity;
        public final double spindexerOutput;
        public final double turretBias;
        public final boolean hoodRetracted;

        Entry(
            double distance,
            double shooterVelocity,
            double spindexerOutput,
            double turretBias,
            boolean hoodRetracted
        ) {
            this.distance = distance;
            this.shooterVelocity = shooterVelocity;
            this.spindexerOutput = spindexerOutput;
            this.turretBias = turretBias;
            this.hoodRetracted = hoodRetracted;
        }

        Entry() {
            this(0, 0, 0, 0, false);
        }
    }

    private final Entry[] distance_buckets = new Entry[] {
        //        new Entry(167, 11_100, 2, 1.5), // untested
        //        new Entry(198, 10_400, 1, 1.5),
        //        new Entry(223, 8_300, 1, 1.45), //untested
        //        new Entry(250, 9_200, 1, 1.4),
        //        new Entry(285, 9_800, 1, 1.35),
        //        new Entry(315, 10_200, 1, 1.25),
        //        new Entry(360, 10_400, 1, 1.25),
        //        new Entry(400, 10_600, 1, 1.25)\
        new Entry(120, 11_400, 2, 1.7, true),
        new Entry(198, 6_500, 1, 1.7, true),
        new Entry(225, 7_400, 1, 1.6, true),
        new Entry(250, 9_600, 1, 1.5, false),
        new Entry(285, 10_000, 1, 1.8, false),
        new Entry(315, 10_100, 1, 1.875, false),
        new Entry(360, 10_400, 1, 1.95, false),
        new Entry(400, 10_600, 1, 2.1, false),
    };

    private final Entry[] zone_buckets = new Entry[] {
        new Entry(1, 7_000, 1, 1.5, false),//1.7),/* 167, 198 were 1.5 */
        new Entry(2, 10_500, 1, 1.8, false),//1.7),
        new Entry(3, 8_700, 1, 1.45, false),//1.6),
        new Entry(4, 10_400, 2, 0.5, false),
        new Entry(5, 9_100, 2, 1.45, true),//1.5),
    };

    public int getZone() {
        return zone;
    }

    public void setZone(int zone) {
        this.zone = zone;
    }

    public double getShooterVelocity(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.shooterVelocity;
            }
        }
        return Shooter.MAX_VELOCITY;
    }

    public double getShooterVelocity() {
        if (USE_ZONES) {
            return getShooterVelocity(zone);
        }
        return getShooterVelocity(camera.getDistance());
    }

    public double getTurretBias(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.turretBias;
            }
        }
        return 1.25;
    }

    public double getTurretBias() {
        if (USE_ZONES) {
            return getTurretBias(zone);
        }
        return getTurretBias(camera.getDistance());
    }

    public double getSpindexerOutput(double distance) {
        for (Entry velocity : buckets) {
            if (distance <= velocity.distance) {
                return velocity.spindexerOutput;
            }
        }
        return 1;
    }

    public double getSpindexerOutput() {
        if (USE_ZONES) {
            return getSpindexerOutput(zone);
        }
        return getSpindexerOutput(camera.getDistance());
    }

    public boolean getHoodRetracted(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.hoodRetracted;
            }
        }
        return false;
    }

    public boolean getHoodRetracted() {
        if (USE_ZONES) {
            return getHoodRetracted(zone);
        }
        return getHoodRetracted(camera.getDistance());
    }
}
