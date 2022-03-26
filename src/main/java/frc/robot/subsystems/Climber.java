// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
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
    if (m_climb_Encoder.getPosition() < 110) {
      climb_motor.set(1);
    } else {
      climb_motor.set(0.0);
    }
  }
  
  public void retract() {
    if (m_climb_Encoder.getPosition() > 2) {
      climb_motor.set(-1);
    } else {
      climb_motor.set(0.0);
    }
  }

  public boolean isActive() {
    return climb_motor.get() > 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");
    builder.addDoubleProperty("Rotations", m_climb_Encoder::getPosition, null);
    builder.addBooleanProperty("Limit switch", limitSwitch::isPressed, null);
  }
}