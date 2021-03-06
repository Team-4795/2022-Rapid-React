// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
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
  private final BreakBeam upperBreakBeam = new BreakBeam(IndexerConstants.UPPER_BREAK_BEAM);
  private final BreakBeam lowerBreakBeam = new BreakBeam(IndexerConstants.LOWER_BREAK_BEAM);
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

  public Color getUpperColor() {
    return colorSensor.getColor();
  }

  public boolean hasUpperBall() {
    return upperBreakBeam.isBroken();
  }

  public boolean hasLowerBall() {
    return lowerBreakBeam.isBroken();
  }

  public boolean isActive() {
    return Math.abs(upperMotor.get()) > 0 || Math.abs(lowerMotor.get()) > 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Indexer");
    builder.addBooleanProperty("Has upper ball", this::hasUpperBall, null);
    builder.addBooleanProperty("Has lower ball", this::hasLowerBall, null);
    builder.addDoubleProperty("Color sensor proximity", colorSensor::getProximity, null);
    builder.addStringProperty("Ball color", () -> {
      Color ballColor = colorSensor.getColor();
      return ballColor == Color.Red ? "red" : (ballColor == Color.Blue ? "blue" : "other");
    }, null);
  }
}