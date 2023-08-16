package com.frc5823.lib.trajectory;

import com.frc5823.lib.geometry.Pose2d;
import com.frc5823.lib.geometry.Twist2d;

public interface IPathFollower {
  Twist2d steer(Pose2d current_pose);

  boolean isDone();
}
