package com.frc5823.lib.controllers;

import com.frc5823.lib.geometry.Rotation2d;
import com.frc5823.lib.utils.Util;

public class HeadingController {
  private boolean isEnabled = false;
  private Rotation2d targetHeading = new Rotation2d();
  private double previousUpdateTimestamp = Double.NaN;
  private double tolerance;
  private final SynchronousPIDF pidController;

  public HeadingController() {
    this(0.0, 0.0, 0.0, 0.0);
  }

  public HeadingController(double kp, double ki, double kd, double errorTolerance) {
    pidController = new SynchronousPIDF(kp, ki, kd);
    this.tolerance = errorTolerance;
    pidController.setInputRange(Math.toRadians(-180.0), Math.toRadians(180.0));
    pidController.setContinuous();
    pidController.setDeadband(this.tolerance);
  }

  public synchronized void enable() {
    if (!isEnabled) {
      isEnabled = true;
      pidController.reset();
      previousUpdateTimestamp = Double.NaN;
    }
  }

  public synchronized void configParmas(double kp, double ki, double kd, double errorTolerance) {
    pidController.setCoefficients(kp, ki, kd);
    pidController.setDeadband(errorTolerance);
  }

  public synchronized void disable() {
    isEnabled = false;
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public synchronized void setInvert(boolean invert) {
    pidController.setInvert(invert);
  }

  public boolean isInverted() {
    return pidController.isInverted();
  }

  public synchronized void setTargetHeading(Rotation2d targetHeading) {
    this.targetHeading = targetHeading;
  }

  public synchronized Rotation2d getTargetHeading() {
    return targetHeading;
  }

  public boolean onTarget() {
    return isEnabled && pidController.onTarget();
  }

  public synchronized double calculate(Rotation2d heading, double timestamp) {
    double output = 0.0;
    if (isEnabled) {
      output =
          pidController.calculate(
              Util.boundAngleToNegative180To180Degrees(
                  heading.rotateBy(targetHeading.inverse()).getRadians()),
              timestamp - previousUpdateTimestamp,
              Double.isNaN(previousUpdateTimestamp));
      previousUpdateTimestamp = timestamp;
    }
    return output;
  }
}
