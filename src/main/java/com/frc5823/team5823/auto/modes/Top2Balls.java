package com.frc5823.team5823.auto.modes;

import com.frc5823.lib.auto.AutoModeEndedException;
import com.frc5823.lib.auto.modes.BaseAutoMode;
import com.frc5823.team5823.Field;
import com.frc5823.team5823.auto.TrajectorySet;
import com.frc5823.team5823.auto.actions.ResetPoseAction;
import com.frc5823.team5823.auto.actions.SetTrajectoryAction;
import com.frc5823.team5823.auto.actions.WaitAction;

public class Top2Balls extends BaseAutoMode {
  private final TrajectorySet trajectorySet = TrajectorySet.getInstance();

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new ResetPoseAction(Field.CriticalWaypoints.TOP_START_ROBOT_POSE, true));
    runAction(new WaitAction(0.2));
    runAction(new SetTrajectoryAction(trajectorySet.topStartToTopBall, -90.0));
  }
}
