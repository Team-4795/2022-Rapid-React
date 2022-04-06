// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  private ShooterPreset preset;
  private final boolean useCV;
  private final boolean useAlignment;
  private long start;
  private boolean reachedAngle;

  public Shoot(Drivebase drivebase, Superstructure superstructure, Vision vision, boolean useAlignment, ShooterPreset ... defaultPreset) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.vision = vision;
    this.useAlignment = useAlignment;
    
    if (defaultPreset.length == 0) {
      useCV = true;
    } else {
      preset = defaultPreset[0];
      useCV = false;
    }

    addRequirements(drivebase, superstructure, vision);
  }

  public Shoot(Drivebase drivebase, Superstructure superstructure, Vision vision, ShooterPreset ... defaultPreset) {
    this(drivebase, superstructure, vision, true, defaultPreset);
  }

  public static ShooterPreset interpolate(double distance) {
    ArrayList<ShooterPreset> presets = new ArrayList<>();

    presets.add(new ShooterPreset(900, 2200, 3));
    presets.add(new ShooterPreset(1950, 1500, 5));
    presets.add(new ShooterPreset(2150, 1500, 6.5));
    presets.add(new ShooterPreset(2500, 1450, 8));
    presets.add(new ShooterPreset(2950, 1350, 10));
    presets.add(new ShooterPreset(3350, 1200, 11));
    presets.add(new ShooterPreset(4100, 800, 12));
    presets.add(new ShooterPreset(4600, 700, 13.5));

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
  public void initialize() {
    if (useCV) preset = interpolate(3);
    drivebase.enableBrakeMode();
    vision.enableLED();
    alliance = DriverStation.getAlliance();
    start = System.currentTimeMillis();
    reachedAngle = false;
  }

  @Override
  public void execute() {
    Color upperColor = superstructure.indexer.getUpperColor();
    boolean isAligned = true;
    
    if (useCV && vision.hasTarget() && !reachedAngle && System.currentTimeMillis() - start < 3000) {
      double distance = vision.getTargetDistance();
      double angle = vision.getTargetAngle();
      double turnSpeed = angle / 50.0;

      preset = interpolate(distance);

      turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.125), turnSpeed), -0.25, 0.25);

      if (useAlignment && Math.abs(angle) > 2) {
        isAligned = false;
      } else {
        drivebase.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(angle)));
        drivebase.setGoalPose(new Pose2d(Units.feetToMeters(distance + 1.5), 0, Rotation2d.fromDegrees(0)));

        reachedAngle = true;
      }

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

    if (isAligned && Math.abs(superstructure.shooter.getMainRPM() - mainRPM) < mainRPM * 0.035 && Math.abs(superstructure.shooter.getTopRPM() - topRPM) < topRPM * 0.035) {
      upperIndexer = 1;
      lowerIndexer = 1;
    }

    if (superstructure.intake.isExtended()) {
      if (superstructure.indexer.hasUpperBall() && superstructure.indexer.hasLowerBall()) superstructure.intake.retract();

      superstructure.intake.setSpeed(0.75);
    } else {
      superstructure.intake.setSpeed(0);
    }

    superstructure.indexer.setIndexerSpeed(upperIndexer, lowerIndexer);
    superstructure.shooter.setShooterRPM(mainRPM, topRPM);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.disableBrakeMode();
    superstructure.intake.setSpeed(0);
    superstructure.indexer.setIndexerSpeed(0, 0);
    superstructure.shooter.setShooterPower(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}