package com.frc5823.team5823.subsystems.vision;

import com.frc5823.lib.drivers.PhotonVision;
import com.frc5823.lib.drivers.VisionResult;
import com.frc5823.lib.geometry.Translation2d;
import com.frc5823.lib.loops.ILoop;
import com.frc5823.lib.loops.ILooper;
import com.frc5823.lib.subsystems.BaseSubsystem;
import com.frc5823.team5823.Config;
import com.frc5823.team5823.Field;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.photonvision.common.hardware.VisionLEDMode;

public class Vision extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(VisionState.ENABLE);
          }

          @Override
          public void onLoop(double timestamp) {
            readPeriodicInputs();
          }

          @Override
          public void onStop(double timestamp) {
            disable();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicInput {
    public boolean isUpdated = false;
    public VisionResult visionResult = new VisionResult();
  }

  @Override
  public void readPeriodicInputs() {
    synchronized (ioLock) {
      periodicInput.isUpdated = true;
      periodicInput.visionResult = photonVision.getVisionResult();
      if (periodicInput.visionResult.hasTarget) {
        periodicInput.visionResult.targetDistance =
            (Field.VISUAL_TARGET_VISUAL_CENTER_HEIGHT - VisionConfig.CAMERA_HEIGHT_METER)
                / Math.tan(
                    periodicInput.visionResult.targetElevation.getRadians()
                        + VisionConfig.CAMERA_ELEVATION.getRadians());

        periodicInput.visionResult.targetOrientation =
            Translation2d.fromPolar(
                periodicInput.visionResult.targetHeading,
                periodicInput.visionResult.targetDistance);
      }
    }
  }
  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private VisionState visionState = VisionState.OFF;

  public synchronized void setState(VisionState new_state) {
    visionState = new_state;
    switch (visionState) {
      case OFF:
        off();
        break;
      case ENABLE:
        enable();
        break;
      case BLINK:
        blink();
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Vision instance = null;

  public static synchronized Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final PeriodicInput periodicInput = new PeriodicInput();

  private final PhotonVision photonVision = new PhotonVision(VisionConfig.CAMERA_NAME);

  private final Object ioLock = new Object();

  public Vision() {
    configSmartDashboard();
    setState(VisionState.OFF);
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  private void enable() {
    photonVision.setLed(VisionLEDMode.kOn);
  }

  private void off() {
    photonVision.setLed(VisionLEDMode.kOff);
  }

  private void blink() {
    photonVision.setLed(VisionLEDMode.kBlink);
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public boolean isUpdated() {
    synchronized (ioLock) {
      if (periodicInput.isUpdated) {
        periodicInput.isUpdated = false;
        return true;
      } else {
        return false;
      }
    }
  }

  /**
   * Get camera centric vision result
   *
   * @return vision result
   */
  public VisionResult getVisionResult() {
    synchronized (ioLock) {
      return periodicInput.visionResult;
    }
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setState(VisionState.OFF);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  private GenericEntry visionStateEntry;

  private GenericEntry hasVisionTargetEntry;
  private GenericEntry targetOrientationEntry;
  private GenericEntry targetElevationEntry;
  private GenericEntry fixedDistanceEntry;
  private GenericEntry cameraLatencyEntry;

  public void configSmartDashboard() {
    var tab = Shuffleboard.getTab("Vision");
    visionStateEntry = tab.add("Vision State", "None").getEntry();
    hasVisionTargetEntry = tab.add("Has Vision Target", false).getEntry();
    targetOrientationEntry = tab.add("Target Orientation", 0.0).getEntry();
    targetElevationEntry = tab.add("Target Elevation", 0.0).getEntry();
    fixedDistanceEntry = tab.add("Fixed Distance", 99.99).getEntry();
    cameraLatencyEntry = tab.add("Camera Latency", 99.99).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      visionStateEntry.setString(visionState.value);
      hasVisionTargetEntry.setBoolean(periodicInput.visionResult.hasTarget);

      if (periodicInput.visionResult.hasTarget) {
        targetOrientationEntry.setDouble(getVisionResult().targetHeading.getDegrees());
        targetElevationEntry.setDouble(getVisionResult().targetElevation.getDegrees());
        fixedDistanceEntry.setDouble(getVisionResult().targetDistance);
        cameraLatencyEntry.setDouble(getVisionResult().latency);
      }
    }
  }
}
