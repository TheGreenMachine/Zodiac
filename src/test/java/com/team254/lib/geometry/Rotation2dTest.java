package com.team254.lib.geometry;

import com.team254.lib.util.Util;
import org.junit.Test;

import static org.junit.Assert.*;

public class Rotation2dTest {

    @Test
    public void distance() {
        var a = Rotation2d.fromDegrees(60);
        var b = Rotation2d.fromDegrees(30);
        var result = Rotation2d.fromRadians(b.distance(a));
        var result2 = Rotation2d.fromDegrees(a.getDegrees() - b.getDegrees());
        System.out.println(result.getDegrees());
        System.out.println(result2.getDegrees());
    }

    @Test
    public void wrap() {
        assertEquals("90", 90, Rotation2d.fromDegrees(90).getDegrees(), Util.kTestEpsilon);
        assertEquals("-90", -90, Rotation2d.fromDegrees(-90).getDegrees(), Util.kTestEpsilon);
        assertEquals("-180", -180, Rotation2d.fromDegrees(-180).getDegrees(), Util.kTestEpsilon);
        System.out.println("-180: " + Rotation2d.fromDegrees(-180).getDegrees());
        System.out.println("-190: " + Rotation2d.fromDegrees(-190).getDegrees());
        System.out.println("180: " + Rotation2d.fromDegrees(180).getDegrees());
        System.out.println("190: " + Rotation2d.fromDegrees(190).getDegrees());
        System.out.println("360: " + Rotation2d.fromDegrees(360).getDegrees());
        System.out.println("-360: " + Rotation2d.fromDegrees(-360).getDegrees());
        System.out.println("370: " + Rotation2d.fromDegrees(370).getDegrees());
        System.out.println("-370: " + Rotation2d.fromDegrees(-370).getDegrees());
    }
}
