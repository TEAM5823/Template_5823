package com.frc5823.team5823.subsystems.vision;

public enum VisionState {
  ENABLE("Enable"),
  OFF("Off"),
  BLINK("Blink");

  public final String value;

  VisionState(String name) {
    this.value = name;
  }
}
