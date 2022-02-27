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
import frc.robot.Constants.Preset;
import frc.robot.sensors.ColorSensor.Color;

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

  public Shoot(Drivebase drivebase, Superstructure superstructure, Shooter shooter, Vision vision) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.shooter = shooter;
    this.vision = vision;

    presets.add(new Preset(400, 2600, 0));
    presets.add(new Preset(400, 3000, 5));
    presets.add(new Preset(600, 3200, 8));
    presets.add(new Preset(3500, 1200, 15));

    addRequirements(drivebase, superstructure, shooter, vision);
  }

  @Override
  public void initialize() {
    stage = Stage.Hold;
    preset = presets.get(0);
    initialDirection = drivebase.getDirection();
    vision.enableLED();
  }

  @Override
  public void execute() {
    Color upperColor = superstructure.indexer.getUpperColor();

    if(vision.hasTarget()) {
      double distance = vision.getTargetDistance();
      double angle = vision.getTargetAngle();
      double driveSpeed = 0;
      double turnSpeed = angle / 100.0;

      for (Preset p : presets) if (Math.abs(distance - p.distance) < Math.abs(distance - preset.distance)) preset = p;

      drivebase.setDirection(-1);

      SmartDashboard.putNumber("distance", distance);
      SmartDashboard.putNumber("preset", preset.distance);

      turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.1), turnSpeed), -0.2, 0.2);

      driveSpeed = MathUtil.clamp((distance - preset.distance) / 10.0, -0.2, 0.2);
      driveSpeed = Math.copySign(Math.max(Math.abs(driveSpeed), 0.1), driveSpeed);

      drivebase.curvatureDrive(Math.abs(distance - preset.distance) > 0.5 ? driveSpeed : 0, Math.abs(angle) > 2 ? turnSpeed : 0, Math.abs(angle) > 2 && Math.abs(distance - preset.distance) < 0.5);
    } else {
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

        if (Math.abs(shooter.getShooterMainRPM() - mainRPM) < mainRPM * 0.05) stage = Stage.Shoot;

        break;
      case Shoot:
        upperIndexer = 0.5;
        lowerIndexer = 0;

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