package com.frc5823.team5823.devices;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticCompressor {
  private static PneumaticCompressor instance = null;

  public static synchronized PneumaticCompressor getInstance() {
    if (instance == null) {
      instance = new PneumaticCompressor();
    }
    return instance;
  }

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public void enable() {
    compressor.enableDigital();
  }

  public void disable() {
    compressor.disable();
  }

  public boolean isEnabled() {
    return compressor.isEnabled();
  }

  public boolean pressureOnTarget() {
    return compressor.getPressureSwitchValue();
  }

  public void update() {}
}
