package com.frc5823.team5823;

import com.frc5823.lib.utils.Util;

/** Final class to store all constants except ID. */
public final class Config {
  // Robot Physical Dimensions (including bumpers)
  public static final double ROBOT_WIDTH = 0.913892;
  public static final double ROBOT_LENGTH = 0.913892;
  public static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0;
  public static final double ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2.0;

  // AHRS
  public static final double INIT_HEADING = 0.0;

  // Lopper
  public static final double LOOPER_CONTROL_PERIOD_SEC = 0.01;
  public static final int LOOPER_CONTROL_DELTA_TIME_MS =
      Util.roundToInt(LOOPER_CONTROL_PERIOD_SEC * 1000.0);
  public static final double LOOPER_VISION_PERIOD_SEC = 0.048;

  // CAN
  public static final int CAN_TIMEOUT_MS = 110;
  public static final int CAN_INSTANT_TIMEOUT_MS = 30;

  // Log
  public static final boolean ENABLE_DEBUG_OUTPUT = true;
}
