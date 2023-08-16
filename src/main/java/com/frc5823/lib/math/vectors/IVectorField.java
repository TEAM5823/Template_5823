package com.frc5823.lib.math.vectors;

import com.frc5823.lib.geometry.Translation2d;

public interface IVectorField {
  Translation2d getVector(Translation2d here);
}
