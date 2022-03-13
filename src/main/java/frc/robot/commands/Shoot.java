// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.Preset;

public class Shoot extends CommandBase {
  private final Drivebase drivebase;
  private final Superstructure superstructure;
  private final Shooter shooter;
  private final Vision vision;
  private double initialDirection;
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
      presets.add(new Preset(1500, 750, 0));
    } else {
      presets.add(defaultPreset[0]);
      useCV = false;
    }

    presets.add(new Preset(400, 3000, 5));
    presets.add(new Preset(1800, 1900, 8));
    presets.add(new Preset(3100, 1200, 12));
    presets.add(new Preset(3300, 1200, 13.5));

    addRequirements(drivebase, superstructure, shooter, vision);
  }

  @Override
  public void initialize() {
    drivebase.enableBrakeMode();
    preset = presets.get(0);
    initialDirection = drivebase.getDirection();
    vision.enableLED();
    start = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    boolean isAligned = true;

    if (vision.hasTarget() && useCV && System.currentTimeMillis() - start < 3000) {
      double distance = vision.getTargetDistance();
      double angle = vision.getTargetAngle() + 2.5;
      double driveSpeed = 0;
      double turnSpeed = angle / 50.0;

      for (Preset p : presets) if (Math.abs(distance - p.distance) < Math.abs(distance - preset.distance)) preset = p;

      drivebase.setDirection(-1);

      SmartDashboard.putNumber("distance", distance);
      SmartDashboard.putNumber("angle", angle);
      SmartDashboard.putNumber("preset", preset.distance);

      turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.12), turnSpeed), -0.25, 0.25);

      driveSpeed = MathUtil.clamp((distance - preset.distance) / 5.0, -0.35, 0.35);
      driveSpeed = Math.copySign(Math.max(Math.abs(driveSpeed), 0.12), driveSpeed);

      if (Math.abs(angle) > 2 || Math.abs(distance - preset.distance) > 0.3) isAligned = false;

      drivebase.curvatureDrive(Math.abs(distance - preset.distance) > 0.3 ? driveSpeed : 0, Math.abs(angle) > 2 ? turnSpeed : 0, Math.abs(angle) > 2 && Math.abs(distance - preset.distance) < 0.3);
    } else {
      drivebase.curvatureDrive(0, 0, false);
    }

    double upperIndexer = 0;
    double lowerIndexer = 0;

    if (Math.abs(shooter.getMainRPM() - preset.mainRPM) < Math.abs(preset.mainRPM) * 0.05 && isAligned && (Math.abs(preset.topRPM) > 500 ? Math.abs(shooter.getTopRPM() - preset.topRPM) < Math.abs(preset.topRPM) * 0.05 : true)) {
      upperIndexer = 0.5;
      lowerIndexer = 1;
    }

    superstructure.indexer.setIndexerSpeed(upperIndexer, lowerIndexer);
    shooter.setShooterRPM(preset.mainRPM, preset.topRPM);
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