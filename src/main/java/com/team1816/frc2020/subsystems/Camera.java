package com.team1816.frc2020.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    private static Camera INSTANCE;

    public static Camera getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Camera();
        }
        return INSTANCE;
    }

    // Components
    private final NetworkTable networkTable;

    // State
    private double deltaXAngle;
    private double distance;

    // Constants
    private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px

    private Camera() {
        networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        networkTable.addEntryListener("center_x", (table, key, entry, value, flags) -> {
            if (value.getDouble() < 0) {
                // Reset deltaX to 0 if contour not detected
                deltaXAngle = 0;
                return;
            }
            var deltaXPixels = (value.getDouble() - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
            this.deltaXAngle = Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) ;
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        networkTable.addEntryListener("distance", (table, key, entry, value, flags) -> {
            // Use most recently available distance if distance not found
            this.distance = value.getDouble();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public double getDeltaXAngle() {
        return deltaXAngle;
    }

    public double getDistance() {
        return distance;
    }
}
