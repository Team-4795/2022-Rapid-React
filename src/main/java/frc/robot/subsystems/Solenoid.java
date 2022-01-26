// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Solenoid extends SubsystemBase {
  /** new double solenoid */
  private DoubleSolenoid mySolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,1); //forward channel, backward channel
  
  public Solenoid() {}
  
  public void extend() { 
      mySolenoid.set(Value.kForward);
  }
  
  public void retract() {
    mySolenoid.set(Value.kReverse);
  }

  public void off(){
    mySolenoid.set(Value.kOff);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}