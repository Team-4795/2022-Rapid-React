// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import frc.robot.sensors.ColorSensor.Color;

public class Shoot extends CommandBase {
  public static final class ShooterPreset {
    public double distance;
    public double topRPM;
    public double mainRPM;

    public ShooterPreset(double t, double m, double d) {
      distance = d;
      topRPM = t;
      mainRPM = m;
    }
  }

  private final Drivebase drivebase;
  private final Superstructure superstructure;
  private final Vision vision;
  private Alliance alliance;
  private double mainRPM, topRPM;
  private ArrayList<ShooterPreset> presets = new ArrayList<>();
  private ShooterPreset preset;
  private boolean useCV = true;
  private long start;

  public Shoot(Drivebase drivebase, Superstructure superstructure, Vision vision, ShooterPreset ... defaultPreset) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.vision = vision;
    
    if (defaultPreset.length == 0) {
      presets.add(new ShooterPreset(1400, 2100, 3));
    } else {
      presets.add(defaultPreset[0]);
      useCV = false;
    }

    presets.add(new ShooterPreset(1650, 1800, 5));
    presets.add(new ShooterPreset(1950, 1600, 6.5));
    presets.add(new ShooterPreset(2300, 1550, 8));
    presets.add(new ShooterPreset(2650, 1500, 10));
    presets.add(new ShooterPreset(3350, 1200, 11));
    presets.add(new ShooterPreset(3900, 950, 12));
    presets.add(new ShooterPreset(4200, 900, 13.5));
    presets.add(new ShooterPreset(4500, 900, 15));

    addRequirements(drivebase, superstructure, vision);
  }

  private ShooterPreset interpolate(double distance) {
    ShooterPreset bottomPreset = presets.get(presets.size() - 1);
    ShooterPreset upperPreset;
    
	  for (int i = presets.size() - 1; i >= 0; i--) {
      if (distance > presets.get(i).distance) {
        bottomPreset = presets.get(i);
        break;
      }
    }

    try {
      upperPreset = presets.get(presets.indexOf(bottomPreset) + 1);
    } catch (IndexOutOfBoundsException e) {
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
  public void initialize() {
    preset = presets.get(0);
    drivebase.enableBrakeMode();
    vision.enableLED();
    alliance = DriverStation.getAlliance();
    start = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    Color upperColor = superstructure.indexer.getUpperColor();
    boolean isAligned = true;

    if (vision.hasTarget() && useCV && System.currentTimeMillis() - start < 3000) {
      double distance = vision.getTargetDistance();
      double angle = -vision.getTargetAngle();
      double turnSpeed = -angle / 50.0;

      preset = interpolate(distance);

      turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.12), turnSpeed), -0.25, 0.25);

      if (Math.abs(angle) > 2) isAligned = false;

      drivebase.curvatureDrive(0, !isAligned ? turnSpeed : 0, true);
    } else {
      drivebase.curvatureDrive(0, 0, false);
    }

    double upperIndexer = 0;
    double lowerIndexer = 0;

    mainRPM = preset.mainRPM;
    topRPM = preset.topRPM;

    if ((upperColor == Color.Red && alliance == Alliance.Blue) || (upperColor == Color.Blue && alliance == Alliance.Red)) {
      mainRPM = 1000;
      topRPM = 1000;
    }

    if (isAligned && Math.abs(superstructure.shooter.getMainRPM() - mainRPM) < mainRPM * 0.05 && Math.abs(superstructure.shooter.getTopRPM() - topRPM) < topRPM * 0.05) {
      upperIndexer = 0.5;
      lowerIndexer = 1;
    }

    superstructure.indexer.setIndexerSpeed(upperIndexer, lowerIndexer);
    superstructure.shooter.setShooterRPM(mainRPM, topRPM);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}