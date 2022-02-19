// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax leftRoller = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushed);
  private CANSparkMax rightRoller = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushed);
  private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  private boolean extended = false;

  public Intake() {}

  public void toggle() {
    if (extended) {
      solenoid.set(Value.kReverse);
    } else {
      solenoid.set(Value.kForward);
    }
    extended = !extended;
  }

  public void setSpeed(double speed) {
    leftRoller.set(speed);
    rightRoller.set(speed);
  }

  public boolean isExtended() {
    return extended;
  }
}