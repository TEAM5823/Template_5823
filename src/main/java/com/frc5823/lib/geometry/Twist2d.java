package com.frc5823.lib.geometry;

import com.frc5823.lib.utils.Util;

import java.text.DecimalFormat;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential
 * calculus" to create new RigidTransform2d's from a Twist2d and visa versa.
 *
 * <p>A Twist can be used to represent a difference between two poses, a velocity, an acceleration,
 * etc.
 */
public class Twist2d {
  protected static final Twist2d kIdentity = new Twist2d(0.0, 0.0, 0.0);

  public static final Twist2d identity() {
    return kIdentity;
  }

  public final double dx;
  public final double dy;
  public final double dtheta; // Radians!

  public Twist2d() {
    this.dx = 0.0;
    this.dy = 0.0;
    this.dtheta = 0.0;
  }

  public Twist2d(double dx, double dy, double dtheta) {
    this.dx = dx;
    this.dy = dy;
    this.dtheta = dtheta;
  }

  public Twist2d(Twist2d other) {
    this.dx = other.dx;
    this.dy = other.dy;
    this.dtheta = other.dtheta;
  }

  public Twist2d scaled(double scale) {
    return new Twist2d(dx * scale, dy * scale, dtheta * scale);
  }

  public double norm() {
    // Common case of dy == 0
    if (dy == 0.0) return Math.abs(dx);
    return Math.hypot(dx, dy);
  }

  public double curvature() {
    if (Math.abs(dtheta) < Util.EPSILON_VALUE && norm() < Util.EPSILON_VALUE) return 0.0;
    return dtheta / norm();
  }

  @Override
  public String toString() {
    final DecimalFormat fmt = new DecimalFormat("#0.000");
    return "("
        + fmt.format(dx)
        + ","
        + fmt.format(dy)
        + ","
        + fmt.format(Math.toDegrees(dtheta))
        + " deg)";
  }
}
