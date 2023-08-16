package com.frc5823.team5823.devices.ahrs;

import com.frc5823.lib.geometry.Rotation2d;

public abstract class BaseAhrs {

  public boolean isReady() {
    return true;
  }

  public abstract Rotation2d getRobotHeading();

  public abstract Rotation2d getRobotAngularVelocity();

  public abstract void setRobotHeading(Rotation2d heading);

  public abstract void resetRobotHeading();
}
