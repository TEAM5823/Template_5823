package com.frc5823.lib.math.vectors;

import com.frc5823.lib.geometry.Translation2d;

import java.util.function.Function;

public class GeneralVectorField extends VectorField {
  public GeneralVectorField(Function<Translation2d, Translation2d> field) {
    field_ = field;
  }

  protected Function<Translation2d, Translation2d> field_;

  public Translation2d getVector(Translation2d here) {
    return field_.apply(here).normalize();
  }
}
