// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.RelativeEncoder;
import java.lang.Math;

public class Climber extends SubsystemBase {
  // private CANSparkMax climb_motor = new CANSparkMax(ClimberConstants.climb_motor, MotorType.kBrushless);
  // private RelativeEncoder m_climb_Encoder;
  // private DigitalInput limitSwitch = new DigitalInput(ClimberConstants.hall_effect_port);
  // private boolean extended = false;

  public Climber() {
    // climb_motor.restoreFactoryDefaults();
    // climb_motor.setIdleMode(IdleMode.kBrake);
    // m_climb_Encoder = climb_motor.getEncoder();
    // climb_motor.setInverted(true);
  }

  public void toggle() {
    // extended = !extended;
  }

  public void extend() {
    // double extend_rotations_needed = (ClimberConstants.stage_length/(ClimberConstants.spool_diameter*Math.PI))*(ClimberConstants.physical_gear*ClimberConstants.versaplanetary)+ClimberConstants.safety_margin;
    // double extension_length = (ClimberConstants.spool_diameter*Math.PI*(m_climb_Encoder.getPosition()/(ClimberConstants.physical_gear * ClimberConstants.versaplanetary)));

    // if(m_climb_Encoder.getPosition() < 130 && m_climb_Encoder.getPosition() >= 0) {
    //   climb_motor.set(0.5);
    // } else {
    //   climb_motor.set(0.0);
    // }
  //  if(ClimberConstants.starting_hook_height+extension_length > ClimberConstants.bar_height){
  //    SmartDashboard.putBoolean("Can climb?", true);
  //  } else{SmartDashboard.putBoolean("Can climb", false);
  }
  
  public void retract() {
    // double retract_rotations_needed = (ClimberConstants.stage_length/(ClimberConstants.spool_diameter*Math.PI))*(ClimberConstants.physical_gear*ClimberConstants.versaplanetary)-ClimberConstants.safety_margin;

    // if(m_climb_Encoder.getPosition()>retract_rotations_needed && limitSwitch.get()==false ) {
    //   climb_motor.set(-1.0);

    //  } else {
    //    climb_motor.set(0.0);
    // if (m_climb_Encoder.getPosition() > 5 && m_climb_Encoder.getPosition() <= 140) {
    //   climb_motor.set(-0.5);
    // } else {
    //   climb_motor.set(0.0);
    // }
  }
   
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Climber Rotations", m_climb_Encoder.getPosition());

    // if (extended) {
    //   extend();
    // } else {
    //   retract();
    // }
  }
}