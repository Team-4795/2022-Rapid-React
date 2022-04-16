// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.sensors.ColorSensor.Color;

public class BallManager extends CommandBase {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private Drivebase drivebase;
  private Alliance alliance;
  private boolean reversed;
  private long startReject;
  private long lastExtend;

  public BallManager(Superstructure superstructure, Drivebase drivebase) {
    this.intake = superstructure.intake;
    this.indexer = superstructure.indexer;
    this.shooter = superstructure.shooter;
    this.drivebase = drivebase;

    addRequirements(superstructure);
  }

  public BallManager(Superstructure superstructure) {
    this(superstructure, null);
  }

  @Override
  public void initialize() {
    alliance = DriverStation.getAlliance();
    reversed = false;
  }

  public void setIntakeReversed(boolean reversed) {
    this.reversed = reversed;
  }

  @Override
  public void execute() {
    if (intake.isExtended()) {
      double intakeSpeed = 0.75 * (reversed ? -1 : 1);
      double upperSpeed = 0.25;
      double lowerSpeed = 1;

      if (indexer.hasUpperBall()) {
        upperSpeed = 0;

        if (indexer.hasLowerBall()) intake.retract();
      }

      intake.setSpeed(intakeSpeed);
      indexer.setIndexerSpeed(upperSpeed, lowerSpeed);

      lastExtend = System.currentTimeMillis();
    } else {
      intake.setSpeed(0);

      if (System.currentTimeMillis() - lastExtend < 2000) {
        double upperSpeed = 0.25;
        double lowerSpeed = 1;

        if (indexer.hasUpperBall()) {
          upperSpeed = 0;

          if (indexer.hasLowerBall()) lowerSpeed = 0;
        }

        indexer.setIndexerSpeed(upperSpeed, lowerSpeed);
      } else {
        indexer.setIndexerSpeed(0, 0);
      }
    }

    Color upperColor = indexer.getUpperColor();

    if (System.currentTimeMillis() - startReject < 1250 || (indexer.hasUpperBall() && ((upperColor == Color.Red && alliance == Alliance.Blue) || (upperColor == Color.Blue && alliance == Alliance.Red)))) {
      shooter.setShooterRPM(1000, 750);

      if (System.currentTimeMillis() - startReject > 2000) startReject = System.currentTimeMillis();

      if (shooter.getMainRPM() > 900) indexer.setIndexerSpeed(0.5, 1);
    } else if (indexer.hasUpperBall() && indexer.hasLowerBall() && drivebase != null) {
      double distance = Units.metersToFeet(drivebase.getGoalPose().getTranslation().getDistance(drivebase.getPose().getTranslation())) - 3.5;
      var preset = Shoot.interpolate(distance);

      if (distance < 14) {
        shooter.setShooterRPM(preset.mainRPM * 0.6, preset.topRPM * 0.6);
      } else {
        shooter.setShooterPower(0, 0);
      }
    } else {
      shooter.setShooterPower(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0);
    indexer.setIndexerSpeed(0, 0);
    shooter.setShooterPower(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}