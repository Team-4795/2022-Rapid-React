// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private ShooterPreset preset;
  private final boolean useCV;
  private final boolean useAlignment;
  private long start;

  public Shoot(Drivebase drivebase, Superstructure superstructure, Vision vision, boolean useAlignment, ShooterPreset ... defaultPreset) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.vision = vision;
    this.useAlignment = useAlignment;
    
    if (defaultPreset.length == 0) {
      superstructure.shooter.addDefaultPreset(new ShooterPreset(900, 2200, 3));
      useCV = true;
    } else {
      superstructure.shooter.addDefaultPreset(defaultPreset[0]);
      useCV = false;
    }

    addRequirements(drivebase, superstructure, vision);
  }

  public Shoot(Drivebase drivebase, Superstructure superstructure, Vision vision, ShooterPreset ... defaultPreset) {
    this(drivebase, superstructure, vision, true, defaultPreset);
  }

  @Override
  public void initialize() {
    preset = superstructure.shooter.getPrests().get(0);
    drivebase.enableBrakeMode();
    vision.enableLED();
    alliance = DriverStation.getAlliance();
    start = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("DB X COORD", drivebase.getRobotCoords().getX());
    SmartDashboard.putNumber("DB Y COORD", drivebase.getRobotCoords().getY());

    Color upperColor = superstructure.indexer.getUpperColor();
    boolean isAligned = true;

    if (vision.hasTarget() && useCV && System.currentTimeMillis() - start < 3000) {
      double distance = vision.getTargetDistance();
      double angle = -vision.getTargetAngle();
      double turnSpeed = -angle / 50.0;

      preset = superstructure.shooter.interpolate(distance);

      turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.12), turnSpeed), -0.25, 0.25);

      if (Math.abs(angle) > 2) isAligned = false;

      drivebase.curvatureDrive(0, !isAligned && useAlignment ? turnSpeed : 0, true);
    } else {
      drivebase.curvatureDrive(0, 0, false);
    }

    double upperIndexer = 0;
    double lowerIndexer = 0;

    mainRPM = preset.mainRPM;
    topRPM = preset.topRPM;

    if ((upperColor == Color.Red && alliance == Alliance.Blue) || (upperColor == Color.Blue && alliance == Alliance.Red)) {
      mainRPM = 1000;
      topRPM = 750;
    }

    if (isAligned && Math.abs(superstructure.shooter.getMainRPM() - mainRPM) < mainRPM * 0.02 && Math.abs(superstructure.shooter.getTopRPM() - topRPM) < topRPM * 0.02) {
      upperIndexer = 0.5;
      lowerIndexer = 1;
    }

    superstructure.indexer.setIndexerSpeed(upperIndexer, lowerIndexer);
    superstructure.shooter.setShooterRPM(mainRPM, topRPM);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.indexer.setIndexerSpeed(0, 0);
    superstructure.shooter.setShooterPower(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}