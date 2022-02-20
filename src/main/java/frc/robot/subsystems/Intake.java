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
  private DoubleSolenoid rollerSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1); 

  public Intake() {
    leftRoller.setInverted(true);
    rightRoller.setInverted(true);

  }

  public void intakeUp() {
    rollerSolenoid.set(Value.kForward);
  }

  public void intakeDown() {
    rollerSolenoid.set(Value.kReverse);
  }

  public void setSpeed(double targetSpeed) {
    leftRoller.set(targetSpeed);
    rightRoller.set(targetSpeed);
  }

  @Override
  public void periodic() {
  } 
}