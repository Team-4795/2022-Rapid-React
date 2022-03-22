// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;

public class ColorSensor {
  public static enum Color {
    Red, Blue, Other
  }

  private ColorSensorV3 colorSensor;

  public ColorSensor() {
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  }

  public Color getColor() {
    Color ballColor = Color.Other;

    try {
      if (getProximity() == 0) colorSensor = new ColorSensorV3(I2C.Port.kMXP);

      var detectedColor = colorSensor.getColor();

      if (getProximity() < 200) {
        ballColor = Color.Other;
      } else if (detectedColor.blue > detectedColor.red) {
        ballColor = Color.Blue;
      } else if (detectedColor.red > detectedColor.blue) {
        ballColor = Color.Red;
      }
    } catch (Exception e) {}

    return ballColor;
  }

  public int getProximity() {
    return colorSensor.getProximity();
  }
}