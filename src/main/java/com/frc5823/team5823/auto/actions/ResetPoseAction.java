package com.frc5823.team5823.auto.actions;

import com.frc5823.lib.auto.actions.BaseRunOnceAction;
import com.frc5823.lib.geometry.Pose2d;
import com.frc5823.team5823.subsystems.swerve.Swerve;

public class ResetPoseAction extends BaseRunOnceAction {
  private final Pose2d newPose;
  private final Swerve swerve;
  private final boolean isSwerveAlign;

  public ResetPoseAction(Pose2d new_pose, boolean is_swerve_align) {
    this.newPose = new_pose;
    isSwerveAlign = is_swerve_align;
    swerve = Swerve.getInstance();
  }

  @Override
  public void runOnce() {
    swerve.setStartingPose(newPose);
    swerve.resetSensors();
    if (isSwerveAlign) {
      swerve.setModulesAlign(newPose.getRotation(), true);
    }
    System.out.println("Swerve pose reset to " + newPose + " !");
  }
}
