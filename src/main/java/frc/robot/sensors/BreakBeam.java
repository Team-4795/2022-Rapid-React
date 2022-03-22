// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BreakBeam {
  private final DigitalInput sensor;

  public BreakBeam(int port) {
    sensor = new DigitalInput(port);
  }

  public boolean isBroken() {
    return !sensor.get();
  }
}