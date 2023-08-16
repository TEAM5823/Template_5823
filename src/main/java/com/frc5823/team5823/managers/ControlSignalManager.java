package com.frc5823.team5823.managers;

import com.frc5823.lib.geometry.Translation2d;
import com.frc5823.lib.io.StatefulXboxController;
import com.frc5823.lib.loops.ILoop;
import com.frc5823.lib.loops.ILooper;
import com.frc5823.lib.utils.Util;
import com.frc5823.team5823.Ports;
import edu.wpi.first.wpilibj.DriverStation;

public class ControlSignalManager {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            resetControlFlags();
          }

          @Override
          public void onLoop(double timestamp) {
            if (!DriverStation.isAutonomousEnabled()) {
              update();
            }
          }

          @Override
          public void onStop(double timestamp) {
            resetControlFlags();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static ControlSignalManager instance = null;

  public static synchronized ControlSignalManager getInstance() {
    if (instance == null) {
      instance = new ControlSignalManager();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final StatefulXboxController driverController;

  private final StatefulXboxController codriverController;

  private ControlSignalManager() {
    driverController =
        new StatefulXboxController(Ports.DriverJoysticks.DRIVER_CONTROLLER_PORT, 0.5);
    codriverController =
        new StatefulXboxController(Ports.DriverJoysticks.CODRIVER_CONTROLLER_PORT, 0.5);

    resetControlFlags();
  }

  /************************************************************************************************
   * Reset *
   ************************************************************************************************/
  public synchronized void resetControlFlags() {}

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public Translation2d getSwerveManualTranslation() {
    return Util.applyRemappedCircularDeadband(
            new Translation2d(-driverController.getRightY(), -driverController.getRightX()),
            0.09375)
        .scale(0.75);
  }

  public double getSwerveManualRotationMagnitude() {
    return Util.applyRemappedDeadband(-driverController.getLeftX(), 0.09375) * 0.27;
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  private synchronized void update() {
    // Use when in manual mode
    driverController.updateButtons();
    codriverController.updateButtons();
  }

  /************************************************************************************************
   * Log *
   ************************************************************************************************/
  public void logToSmartDashBoard() {}
}
