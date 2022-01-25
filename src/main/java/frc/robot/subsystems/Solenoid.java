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

  private boolean isExtended = false;
  
  public Solenoid() {}
  
  public void run() { 
    if (!isExtended) {
      mySolenoid.set(Value.kForward);
    } else {
      mySolenoid.set(Value.kReverse);
    }
  }
  
  public void off() {
    if (mySolenoid.get() == DoubleSolenoid.Value.kForward) {
      isExtended = true;
    } else if (mySolenoid.get() == DoubleSolenoid.Value.kReverse) {
      isExtended = false;
    }
    mySolenoid.set(Value.kOff);
  }

  public boolean getExtended() {
    return isExtended;
  }

  public void setExtended(boolean Extended) {
    isExtended = Extended;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
