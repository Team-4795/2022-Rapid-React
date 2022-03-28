// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot.ShooterPreset;

public class Shooter extends SubsystemBase {
  private final TalonFX FlywheelMain = new TalonFX(ShooterConstants.FLYWHEEL_MAIN_TALON);
  private final TalonFX FlywheelTop = new TalonFX(ShooterConstants.FLYWHEEL_TOP_TALON);
  private double targetRPM;
  private ArrayList<ShooterPreset> presets;

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
    FlywheelMain.config_kP(0, 0.07, 0);
    FlywheelMain.config_kI(0, 0.0001, 0);
    FlywheelMain.config_IntegralZone(0, 150.0 / (600.0) * 2048.0);

    FlywheelTop.config_kF(0, 0.0512, 0);
    FlywheelTop.config_kP(0, 0.07, 0);
    FlywheelTop.config_kI(0, 0.0001, 0);
    FlywheelTop.config_IntegralZone(0, 150.0 / (600.0) * 2048.0);

    presets.add(new ShooterPreset(1950, 1500, 5));
    presets.add(new ShooterPreset(2150, 1500, 6.5));
    presets.add(new ShooterPreset(2500, 1450, 8));
    presets.add(new ShooterPreset(2950, 1350, 10));
    presets.add(new ShooterPreset(3350, 1200, 11));
    presets.add(new ShooterPreset(4100, 800, 12));
    presets.add(new ShooterPreset(4900, 700, 13.5));
  }

  public void addDefaultPreset(ShooterPreset p) {
    presets.add(0, p);
  }

  public ArrayList<ShooterPreset> getPrests() {
    return presets;
  }
  
  public void setShooterPower(double speedMain, double speedTop) {
    targetRPM = 0;

    FlywheelMain.set(ControlMode.PercentOutput, speedMain);
    FlywheelTop.set(ControlMode.PercentOutput, speedTop);
  }

  public void setShooterRPM(double speedMain, double speedTop) {
    targetRPM = speedMain;
    // 2048 ticks per revolution, ticks per .10 second, 1 / 2048 * 60
    double speed_FalconUnits1 = speedMain / (600.0) * 2048.0;
    double speed_FalconUnits2 = speedTop / (600.0) * 2048.0;

    if (Math.abs(getMainRPM()) < Math.abs(speedMain) * 1.1) {
      FlywheelMain.set(TalonFXControlMode.Velocity, speed_FalconUnits1);
    } else {
      FlywheelMain.set(ControlMode.PercentOutput, 0);
    }

    if (Math.abs(getTopRPM()) < Math.abs(speedTop) * 1.1) {
      FlywheelTop.set(TalonFXControlMode.Velocity, speed_FalconUnits2);
    } else {
      FlywheelTop.set(ControlMode.PercentOutput, 0);
    }
  }

  public double getMainRPM() {
    return (FlywheelMain.getSelectedSensorVelocity()) / 2048.0 * 600;
  }

  public double getTopRPM() {
    return (FlywheelTop.getSelectedSensorVelocity()) / 2048.0 * 600;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public ShooterPreset interpolate(double distance) {
    ShooterPreset bottomPreset = presets.get(presets.size() - 1);
    ShooterPreset upperPreset;
    
	  for (ShooterPreset p : presets) {
      if (distance - p.distance < 0) {
        bottomPreset = p;
        break;
      }
    }

    try {
      bottomPreset = presets.get(presets.indexOf(bottomPreset) - 1);
      upperPreset = presets.get(presets.indexOf(bottomPreset) + 1);
    } catch (IndexOutOfBoundsException e) {
      if (distance > presets.get(presets.size() - 1).distance) {
        bottomPreset = presets.get(presets.size() - 1);
      }
      upperPreset = bottomPreset;
    }

    double topRPMDifference = upperPreset.topRPM - bottomPreset.topRPM;
    double mainRPMDifference = upperPreset.mainRPM - bottomPreset.mainRPM;
    double dist = upperPreset.distance - bottomPreset.distance;

    if (dist == 0) dist = bottomPreset.distance;

    double percentage = (distance - bottomPreset.distance) / dist;

    double topRPM = percentage * topRPMDifference + bottomPreset.topRPM;
    double mainRPM = percentage * mainRPMDifference + bottomPreset.mainRPM;
    
    return new ShooterPreset(topRPM, mainRPM, distance);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("Main RPM", this::getMainRPM, null);
    builder.addDoubleProperty("Top RPM", this::getTopRPM, null);
  }
}