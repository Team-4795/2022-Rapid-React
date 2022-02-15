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
  private CANSparkMax roller = new CANSparkMax(IntakeConstants.ROLLER, MotorType.kBrushless);
  private DoubleSolenoid rollerSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1); 
  private double speed = 0;
  private boolean extended = false;

  public Intake() {}

  public void toggleIntake() {
    if (extended) {
      rollerSolenoid.set(Value.kReverse);
      roller.set(0);
    } else {
      rollerSolenoid.set(Value.kForward);
      roller.set(.5);
    }
    extended = !extended;
  }

  public void setSpeed(double targetSpeed) {
    speed = targetSpeed;
    roller.set(speed);
  }

  @Override
  public void periodic() {
    roller.set(speed);
  } 
}