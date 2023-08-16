package com.frc5823.lib.trajectory;

import com.frc5823.lib.geometry.IState;

public interface ITrajectoryView<S extends IState<S>> {
  TrajectorySamplePoint<S> sample(final double interpolant);

  double first_interpolant();

  double last_interpolant();

  Trajectory<S> trajectory();
}
