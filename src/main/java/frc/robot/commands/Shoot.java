// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.sensors.Colors;

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
  private double mainRPM, topRPM, upperIndexer, lowerIndexer;

  public Shoot(Drivebase drivebase, Superstructure superstructure, Shooter shooter, Vision vision) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.shooter = shooter;
    this.vision = vision;

    addRequirements(drivebase, superstructure, shooter, vision);
  }

  @Override
  public void initialize() {
    stage = Stage.Hold;
    vision.enableLED();
  }

  @Override
  public void execute() {
    double distance = 0;
    Colors upperColor = superstructure.indexer.getUpperColor();

    if(vision.hasTarget()) {
      distance = vision.getTargetDistance();
      double angle = vision.getTargetAngle();
      double turnSpeed = -angle / 400.0;
      
      turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.035), turnSpeed), -0.15, 0.15);

      if(Math.abs(angle) > 2) {
        drivebase.curvatureDrive(0, turnSpeed, true);
      } else {
        drivebase.curvatureDrive(0, 0, false);
      }
    }

    switch (stage) {
      case Hold:
        upperIndexer = 0;
        lowerIndexer = 0;

        if(distance > 12) {
          mainRPM = 6000;
          topRPM = 6000;
        } else if(distance > 5) {
          mainRPM = 5500;
          topRPM = 4000;
        } else {
          mainRPM = 5000;
          topRPM = 2000;
        }
    
        if(upperColor == Colors.Red && alliance == Alliance.Blue) {
          mainRPM = 1000;
          topRPM = 5000;
        }
    
        if(upperColor == Colors.Blue && alliance == Alliance.Red) {
          mainRPM = 1000;
          topRPM = 5000;
        }

        if (Math.abs(shooter.getShooterMainRPM() - mainRPM) < mainRPM * 0.05) stage = Stage.Shoot;

        break;
      case Shoot:
        upperIndexer = 0.5;
        lowerIndexer = 0;

        if (upperColor == Colors.Other) stage = Stage.Feed;

        break;
      case Feed:
        upperIndexer = 0.25;
        lowerIndexer = 1;

        if (upperColor != Colors.Other) stage = Stage.Hold;

        break;
    }

    superstructure.indexer.setIndexerSpeed(upperIndexer, lowerIndexer);
    shooter.setShooterRPM(mainRPM, topRPM);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}