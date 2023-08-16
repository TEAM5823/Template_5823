package com.frc5823.lib.geometry;

import com.frc5823.lib.utils.IInterpolable;
import com.frc5823.lib.utils.ICSVWritable;

public interface IState<S> extends IInterpolable<S>, ICSVWritable {
  double distance(final S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}
