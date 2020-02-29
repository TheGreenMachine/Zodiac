package com.team254.lib.geometry;

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
}
