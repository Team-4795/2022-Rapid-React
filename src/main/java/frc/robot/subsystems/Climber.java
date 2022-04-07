// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax climb_motor = new CANSparkMax(ClimberConstants.CLIMB_MOTOR, MotorType.kBrushless);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.FORWARD_CHANNEL, ClimberConstants.REVERSE_CHANNEL);
  private final RelativeEncoder climb_encoder;

  public Climber() {
    climb_motor.restoreFactoryDefaults();
    climb_motor.setIdleMode(IdleMode.kBrake);
    climb_motor.setInverted(false);

    climb_encoder = climb_motor.getEncoder();

    solenoid.set(Value.kForward);
  }

  public void setPower(double power) {
    climb_motor.set(power);
  }

  public void resetEncoder() {
    climb_encoder.setPosition(0);
  }

  public void extend() {
    if (climb_encoder.getPosition() < 110) {
      climb_motor.set(1);
    } else {
      climb_motor.set(0.0);
    }
  }
  
  public void retract() {
    if (climb_encoder.getPosition() > -10) {
      climb_motor.set(-1);
    } else {
      climb_motor.set(0.0);
    }
  }

  public void tilt() {
    solenoid.set(Value.kReverse);
  }

  public void untilt() {
    solenoid.set(Value.kForward);
  }
  
  public boolean isActive() {
    return climb_motor.get() > 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");
    builder.addDoubleProperty("Rotations", climb_encoder::getPosition, null);
  }
}