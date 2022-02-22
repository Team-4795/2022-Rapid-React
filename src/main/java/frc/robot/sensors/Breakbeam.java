// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Breakbeam {
  private DigitalInput sensor;

  public Breakbeam() {
    sensor = new DigitalInput(1);
  }

  public boolean isBroken() {
    SmartDashboard.putBoolean("beam", !sensor.get());
    return !sensor.get();
  }
}