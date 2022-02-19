// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Colors;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.ColorSensorV2;
import frc.robot.Constants.IndexerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Indexer extends SubsystemBase {
  private CANSparkMax upperMotor = new CANSparkMax(IndexerConstants.INDEXER_UPPER, MotorType.kBrushless);
  private CANSparkMax lowerMotor = new CANSparkMax(IndexerConstants.INDEXER_LOWER, MotorType.kBrushed);
  private ColorSensorV2 upperColorSensor = new ColorSensorV2();
  private ColorSensor lowerColorSensor = new ColorSensor();

  public Indexer() {  
    upperMotor.restoreFactoryDefaults();
    lowerMotor.restoreFactoryDefaults();

    upperMotor.setIdleMode(IdleMode.kBrake);
    lowerMotor.setIdleMode(IdleMode.kBrake);

    upperMotor.enableVoltageCompensation(12);
    lowerMotor.enableVoltageCompensation(12);
  }

  public void setIndexerSpeed(double upperSpeed, double lowerSpeed) {
    upperMotor.set(upperSpeed);
    lowerMotor.set(lowerSpeed);
  }

  public Colors getUpperColor() {
    return upperColorSensor.getBallColor();
  }

  public Colors getLowerColor() {
    return lowerColorSensor.getBallColor();
  }
}