package com.frc5823.lib.geometry;

public interface ICurvature<S> extends IState<S> {
  double getCurvature();

  double getDCurvatureDs();
}
