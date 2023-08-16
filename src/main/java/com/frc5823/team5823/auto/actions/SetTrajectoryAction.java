package com.frc5823.team5823.auto.actions;

import com.frc5823.lib.auto.actions.BaseAction;
import com.frc5823.lib.geometry.Pose2dWithCurvature;
import com.frc5823.lib.trajectory.Trajectory;
import com.frc5823.lib.trajectory.timing.TimedState;
import com.frc5823.lib.utils.Util;
import com.frc5823.team5823.subsystems.swerve.Swerve;
import com.frc5823.team5823.subsystems.swerve.SwerveState;

public class SetTrajectoryAction extends BaseAction {
  private final Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
  private final double goalHeading;
  private final boolean isLastTrajectory;
  private final Swerve swerve;

  public SetTrajectoryAction(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goal_heading) {
    this(trajectory, goal_heading, false);
  }

  public SetTrajectoryAction(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
      double goal_heading,
      boolean is_last_trajectory) {
    this.trajectory = trajectory;
    goalHeading = goal_heading;
    isLastTrajectory = is_last_trajectory;
    swerve = Swerve.getInstance();
  }

  @Override
  public void start() {
    swerve.setTrajectory(trajectory, goalHeading);
    System.out.println("Trajectory start!");
  }

  @Override
  public void update() {}

  @Override
  public void done() {}

  @Override
  public boolean isFinished() {
    if (isLastTrajectory) {
      var currentTranslation = swerve.getPose().getTranslation();
      var lastTargetTranslation = trajectory.getLastState().state().getPose().getTranslation();

      if (Util.epsilonEquals(currentTranslation.x(), lastTargetTranslation.x(), 0.5)
          && Util.epsilonEquals(currentTranslation.y(), lastTargetTranslation.y(), 0.3)) {
        swerve.setState(SwerveState.DISABLE);
        return true;
      } else {
        return false;
      }
    } else if (swerve.isDoneWithTrajectory()) {
      System.out.println("Trajectory finished!");
      swerve.disableModules();
      return true;
    } else {
      return false;
    }
  }
}
