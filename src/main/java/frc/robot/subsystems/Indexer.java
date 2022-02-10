// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class Indexer extends SubsystemBase {
  
  private CANSparkMax upperMotor = new CANSparkMax(IndexerConstants.INDEXER_UPPER, MotorType.kBrushless);
  private CANSparkMax lowerMotor = new CANSparkMax(IndexerConstants.INDEXER_LOWER, MotorType.kBrushed);

  PIDController pid = new PIDController(IndexerConstants.kP, IndexerConstants.kI, IndexerConstants.kD);
 
  private double targetVelocity = 0;

  private RelativeEncoder m_upperEncoder;

  public Indexer() {  

    upperMotor.restoreFactoryDefaults();
    lowerMotor.restoreFactoryDefaults();

    upperMotor.setIdleMode(IdleMode.kCoast);
    lowerMotor.setIdleMode(IdleMode.kCoast);

    m_upperEncoder = upperMotor.getEncoder();
  }

  public double getUpperEncoder() {
    return m_upperEncoder.getVelocity();
  }

  public void SetIndexerSpeed(double upperSpeed, double lowerSpeed) {
    targetVelocity = upperSpeed;
    lowerMotor.set(lowerSpeed);
  }
/*
  public void setIndexerPower(double upperSpeed, double lowerSpeed) {
    upperMotor.set(upperSpeed);
    lowerMotor.set(lowerSpeed);
  }

  public void setIndexerSpeed(double upperSpeed, double lowerSpeed) {
    if (getUpperEncoder() >= upperSpeed){
      setIndexerPower(0, lowerSpeed);
    } else {
      setIndexerPower(upperSpeed, lowerSpeed);
    }
  }
*/
  @Override
  public void periodic() {
    upperMotor.set(pid.calculate(m_upperEncoder.getVelocity(), targetVelocity));
  }
}