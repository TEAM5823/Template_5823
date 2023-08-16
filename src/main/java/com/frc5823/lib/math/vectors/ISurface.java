package com.frc5823.lib.math.vectors;

import com.frc5823.lib.geometry.Translation2d;

import java.util.function.Function;

public interface ISurface {
  Function<Translation2d, Double> f();

  Function<Translation2d, Double> dfdx();

  Function<Translation2d, Double> dfdy();
}
