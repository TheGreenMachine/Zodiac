package com.team1816.frc2020.subsystems;

import junit.framework.TestCase;

public class TurretTest extends TestCase {

    private Turret target;

    public void setUp() throws Exception {
        target = Turret.getInstance();
    }

    public void testSetTurretPosition() {
        target.setTurretAngle(Turret.CARDINAL_SOUTH);
    }

    public void testLockTurret() {
    }
}
