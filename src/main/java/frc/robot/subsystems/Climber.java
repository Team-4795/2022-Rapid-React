// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax climb_motor = new CANSparkMax(ClimberConstants.CLIMB_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_climb_Encoder;
  private SparkMaxLimitSwitch limitSwitch;

  public Climber() {
    climb_motor.restoreFactoryDefaults();
    climb_motor.setIdleMode(IdleMode.kBrake);
    m_climb_Encoder = climb_motor.getEncoder();
    climb_motor.setInverted(false);
    limitSwitch = climb_motor.getReverseLimitSwitch(Type.kNormallyOpen);
    limitSwitch.enableLimitSwitch(true);
  }

  public void setPower(double power) {
    climb_motor.set(power);
  }

  public void resetEncoder() {
    m_climb_Encoder.setPosition(0);
  }

  public void extend() {
    if (m_climb_Encoder.getPosition() < 105 && m_climb_Encoder.getPosition() >= 0) {
      climb_motor.set(0.5);
    } else {
      climb_motor.set(0.0);
    }
  }
  
  public void retract() {
    if (m_climb_Encoder.getPosition() > 5 && m_climb_Encoder.getPosition() <= 115) {
      climb_motor.set(-1);
    } else {
      climb_motor.set(0.0);
    }
  }
   
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Rotations", m_climb_Encoder.getPosition());
    SmartDashboard.putBoolean("Limit switch", limitSwitch.isPressed());
  }
}