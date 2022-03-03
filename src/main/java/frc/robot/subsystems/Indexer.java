// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.ColorSensor.Color;
import frc.robot.sensors.BreakBeam;
import frc.robot.sensors.ColorSensor;
import frc.robot.Constants.IndexerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Indexer extends SubsystemBase {
  private final CANSparkMax upperMotor = new CANSparkMax(IndexerConstants.INDEXER_UPPER, MotorType.kBrushless);
  private final CANSparkMax lowerMotor = new CANSparkMax(IndexerConstants.INDEXER_LOWER, MotorType.kBrushed);
  private final BreakBeam breakBeam = new BreakBeam(IndexerConstants.BREAK_BEAM_PORT);
  private final ColorSensor colorSensor = new ColorSensor();

  public Indexer() {
    upperMotor.restoreFactoryDefaults();
    lowerMotor.restoreFactoryDefaults();

    upperMotor.setIdleMode(IdleMode.kBrake);
    lowerMotor.setIdleMode(IdleMode.kBrake);

    lowerMotor.setInverted(true);
    upperMotor.setInverted(true);

    upperMotor.enableVoltageCompensation(12);
    lowerMotor.enableVoltageCompensation(12);

    upperMotor.setSmartCurrentLimit(IndexerConstants.CURRENT_LIMIT);
  }

  public void setIndexerSpeed(double upperSpeed, double lowerSpeed) {
    upperMotor.set(upperSpeed);
    lowerMotor.set(lowerSpeed);
  }

  public Color getLowerColor() {
    return colorSensor.getColor();
  }

  public boolean hasUpperBall() {
    return breakBeam.isBroken();
  }

  public boolean hasLowerBall() {
    return colorSensor.getProximity() > 500;
  }
}