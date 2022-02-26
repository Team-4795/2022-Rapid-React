// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX FlywheelMain = new TalonFX(ShooterConstants.FLYWHEEL_MAIN_TALON);
  private final TalonFX FlywheelTop = new TalonFX(ShooterConstants.FLYWHEEL_TOP_TALON);

  public Shooter() {
    FlywheelMain.configFactoryDefault();
    FlywheelTop.configFactoryDefault();
    
    FlywheelMain.configVoltageCompSaturation(12);
    FlywheelMain.enableVoltageCompensation(true);

    FlywheelTop.configVoltageCompSaturation(12);
    FlywheelTop.enableVoltageCompensation(true);

    FlywheelMain.setInverted(false);
    FlywheelTop.setInverted(false);

    FlywheelMain.setNeutralMode(NeutralMode.Coast);
    FlywheelTop.setNeutralMode(NeutralMode.Coast);
    
    FlywheelMain.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    FlywheelTop.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    FlywheelMain.config_kF(0, 0.0512, 0);
    FlywheelMain.config_kP(0, 0.04, 0);

    FlywheelTop.config_kF(0, 0.0512, 0);
    FlywheelTop.config_kP(0, 0.04, 0);
  }

  public void setShooterPower(double speedMain, double speedTop) {
    FlywheelMain.set(ControlMode.PercentOutput, speedMain);
    FlywheelTop.set(ControlMode.PercentOutput, speedTop);
  }

  public void setShooterRPM(double speedMain, double speedTop) {
    // 2048 ticks per revolution, ticks per .10 second, 1 / 2048 * 60
    double speed_FalconUnits1 = speedMain / (600.0) * 2048.0;
    double speed_FalconUnits2 = speedTop / (600.0) * 2048.0;

    if (getShooterMainRPM() < speedMain * 1.1) {
      FlywheelMain.set(TalonFXControlMode.Velocity, speed_FalconUnits1);
    } else {
      FlywheelMain.set(ControlMode.PercentOutput, 0);
    }

    if (getShooterTopRPM() < speedTop * 1.1) {
      FlywheelTop.set(TalonFXControlMode.Velocity, speed_FalconUnits2);
    } else {
      FlywheelTop.set(ControlMode.PercentOutput, 0);
    }
  }

  public double getShooterMainRPM() {
    return (FlywheelMain.getSelectedSensorVelocity()) / 2048.0 * 600;
  }

  public double getShooterTopRPM() {
    return (FlywheelTop.getSelectedSensorVelocity()) / 2048.0 * 600;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter Main RPM", getShooterMainRPM());
    SmartDashboard.putNumber("shooter Top RPM", getShooterTopRPM());
  }
}