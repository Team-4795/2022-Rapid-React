// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.sensors.ColorSensor.Color;
import frc.robot.Constants.Preset;

enum Stage {
  Hold, Shoot, Feed
}

public class Shoot extends CommandBase {
  private final Drivebase drivebase;
  private final Superstructure superstructure;
  private final Shooter shooter;
  private final Vision vision;
  private final Alliance alliance = DriverStation.getAlliance();
  private Stage stage;
  private double mainRPM, topRPM, upperIndexer, lowerIndexer, initialDirection;
  private ArrayList<Preset> presets = new ArrayList<>();
  private Preset preset;
  private boolean useCV = true;
  private long start;

  public Shoot(Drivebase drivebase, Superstructure superstructure, Shooter shooter, Vision vision, Preset ... defaultPreset) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.shooter = shooter;
    this.vision = vision;
    
    if (defaultPreset.length == 0) {
      presets.add(new Preset(1500, 1800, 5));
    } else {
      presets.add(defaultPreset[0]);
      useCV = false;
    }

    presets.add(new Preset(1900, 1700, 8));
    presets.add(new Preset(3100, 1200, 12));
    presets.add(new Preset(3800, 1000, 15));

    addRequirements(drivebase, superstructure, shooter, vision);
  }

  @Override
  public void initialize() {
    stage = Stage.Hold;
    preset = presets.get(0);
    drivebase.enableBrakeMode();
    initialDirection = drivebase.getDirection();
    vision.enableLED();
    start = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    Color upperColor = superstructure.indexer.getUpperColor();
    boolean isAligned = true;

    if (vision.hasTarget() && useCV && System.currentTimeMillis() - start < 3000) {
      double distance = vision.getTargetDistance();
      double angle = -vision.getTargetAngle();
      double driveSpeed = 0;
      double turnSpeed = -angle / 50.0;

      for (Preset p : presets) if (Math.abs(distance - p.distance) < Math.abs(distance - preset.distance)) preset = p;

      drivebase.setDirection(-1);

      SmartDashboard.putNumber("preset", preset.distance);

      turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.12), turnSpeed), -0.25, 0.25);

      driveSpeed = MathUtil.clamp((distance - preset.distance) / 5.0, -0.35, 0.35);
      driveSpeed = Math.copySign(Math.max(Math.abs(driveSpeed), 0.12), driveSpeed);

      if (Math.abs(angle) > 2 || Math.abs(distance - preset.distance) > 0.3) isAligned = false;

      drivebase.curvatureDrive(Math.abs(distance - preset.distance) > 0.3 ? driveSpeed : 0, Math.abs(angle) > 2 ? turnSpeed : 0, Math.abs(angle) > 2 && Math.abs(distance - preset.distance) < 0.3);
    } else {
      if (SmartDashboard.getBoolean("USE INTERPOLATION", false)) {
        preset = shooter.interpolate(SmartDashboard.putNumber("CV DISTANCE (TEST), 5"), presets); //ADDED BEFORE FUNCTION DEF - WILL BE FIXED IN LATER COMMIT
      }
        
      drivebase.curvatureDrive(0, 0, false);
    }

    switch (stage) {
      case Hold:
        upperIndexer = 0;
        lowerIndexer = 0;

        mainRPM = preset.mainRPM;
        topRPM = preset.topRPM;
    
        if (upperColor == Color.Red && alliance == Alliance.Blue) {
          mainRPM = 1000;
          topRPM = 1000;
        }
    
        if (upperColor == Color.Blue && alliance == Alliance.Red) {
          mainRPM = 1000;
          topRPM = 1000;
        }

        if (isAligned && Math.abs(shooter.getMainRPM() - mainRPM) < mainRPM * 0.05 && Math.abs(shooter.getTopRPM() - topRPM) < topRPM * 0.05) stage = Stage.Shoot;

        break;
      case Shoot:
        upperIndexer = 0.5;
        lowerIndexer = 0.5;

        if (upperColor == Color.Other) stage = Stage.Feed;

        break;
      case Feed:
        upperIndexer = 0.25;
        lowerIndexer = 1;

        if (upperColor != Color.Other) stage = Stage.Hold;

        break;
    }

    superstructure.indexer.setIndexerSpeed(upperIndexer, lowerIndexer);
    shooter.setShooterRPM(mainRPM, topRPM);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setDirection(initialDirection);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
