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

  public String getBallColor() {
    Color detectedColor = getColor();
    String ballColor = "other";

    if (getIR() < 20) {
      ballColor = "other";
    } else if (detectedColor.blue > detectedColor.red) {
      ballColor = "blue";
    } else if (detectedColor.red > detectedColor.blue) {
      ballColor = "red";
    } else {
      ballColor = "other";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Detected Color: ", ballColor);

    return ballColor;
  }

  public double getIR() {
    return m_colorSensor.getIR();
  }

  public double getProxy() {
    return m_colorSensor.getProximity();
  }

  public Color getColor(){
    return m_colorSensor.getColor();
  }

  public void init() {
    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  }
}