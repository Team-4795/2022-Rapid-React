// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;

public class ColorSensor {
  public static enum Color {
    Red, Blue, Other
  }

  private final ColorSensorV3 colorSensor;

  public ColorSensor() {
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  }

  public Color getColor() {
    Color ballColor = Color.Other;

    try {
      var detectedColor = colorSensor.getColor();

      if (getProximity() < 150) {
        ballColor = Color.Other;
      } else if (detectedColor.blue > detectedColor.red) {
        ballColor = Color.Blue;
      } else if (detectedColor.red > detectedColor.blue) {
        ballColor = Color.Red;
      }

      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putString("Detected Color: ", ballColor == Color.Red ? "red" : "blue");
    } catch (Exception exception) {
      SmartDashboard.putString("Detected Color: ", "error");
    }

    return ballColor;
  }

  public int getProximity() {
    SmartDashboard.putNumber("Prox", colorSensor.getProximity());
    return colorSensor.getProximity();
  }
}