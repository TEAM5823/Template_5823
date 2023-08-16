package com.frc5823.lib.math.vectors;

import com.frc5823.lib.geometry.Translation2d;

public class ConstantVectorField extends VectorField {
  public ConstantVectorField(Translation2d whichWay) {
    thatWay = whichWay.normalize();
  }

  protected Translation2d thatWay;

  public Translation2d getVector(Translation2d here) {
    return thatWay;
  }
}
