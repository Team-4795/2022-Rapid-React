// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.sensors.BreakBeam;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftRoller = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushed);
  private final CANSparkMax rightRoller = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushed);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(0, PneumaticsModuleType.REVPH, IntakeConstants.FORWARD_CHANNEL, IntakeConstants.REVERSE_CHANNEL);
  private final BreakBeam breakBeam = new BreakBeam(IntakeConstants.BREAK_BEAM);
  private boolean extended = false;

  public Intake() {
    leftRoller.restoreFactoryDefaults();
    rightRoller.restoreFactoryDefaults();

    leftRoller.setInverted(false);
    rightRoller.setInverted(true);

    leftRoller.setIdleMode(IdleMode.kCoast);
    rightRoller.setIdleMode(IdleMode.kCoast);
    
    solenoid.set(Value.kReverse);
  }

  public void toggle() {
    if (extended) {
      solenoid.set(Value.kReverse);
      setSpeed(0);
    } else {
      solenoid.set(Value.kForward);
    }
    extended = !extended;
  }

  public void deploy() {
    solenoid.set(Value.kForward);
    extended = true;
  }

  public void retract() {
    solenoid.set(Value.kReverse);
    extended = false;
  }

  public void setSpeed(double speed) {
    if (extended) {
      leftRoller.set(speed);
      rightRoller.set(speed);
    }
  }

  public boolean isExtended() {
    return extended;
  }

  public boolean hasBall() {
    return breakBeam.isBroken();
  }
}