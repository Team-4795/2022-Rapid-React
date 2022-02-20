// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class ColorSensor {
  private ColorSensorV3 m_colorSensor;

  public ColorSensor() {
    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  }

  public Colors getBallColor() {
    Color detectedColor = m_colorSensor.getColor();
    Colors ballColor = Colors.Other;

    if (m_colorSensor.getIR() < 20) {
      ballColor = Colors.Other;
    } else if (detectedColor.blue > detectedColor.red) {
      ballColor = Colors.Blue;
    } else if (detectedColor.red > detectedColor.blue) {
      ballColor = Colors.Red;
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Detected Color: ", ballColor == Colors.Red ? "red" : "blue");

    return ballColor;
  }
}