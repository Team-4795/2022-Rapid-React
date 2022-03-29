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
import frc.robot.commands.Shoot.ShooterPreset;
import frc.robot.sensors.ColorSensor.Color;

public class BallManager extends CommandBase {
  private final Drivebase drivebase;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private Alliance alliance;
  private boolean reversed;
  private long startReject;

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

        if (indexer.hasLowerBall()) intake.toggle();
      }

      intake.setSpeed(intakeSpeed);
      indexer.setIndexerSpeed(upperSpeed, lowerSpeed);
    } else if (drivebase != null && drivebase.getGoalPose() != null && indexer.hasUpperBall() && indexer.hasLowerBall()) {
      double distance = Units.metersToFeet(drivebase.getGoalPose().getTranslation().getDistance(drivebase.getPose().getTranslation()));
      ShooterPreset preset = Shoot.interpolate(distance);

      intake.setSpeed(0);
      indexer.setIndexerSpeed(0, 0);
      shooter.setShooterRPM(preset.mainRPM * 0.8, preset.topRPM * 0.8);
    } else {
      intake.setSpeed(0);
      indexer.setIndexerSpeed(0, 0);
    }

    Color upperColor = indexer.getUpperColor();

    if (System.currentTimeMillis() - startReject < 1250 || (indexer.hasUpperBall() && ((upperColor == Color.Red && alliance == Alliance.Blue) || (upperColor == Color.Blue && alliance == Alliance.Red)))) {
      shooter.setShooterRPM(1000, 750);
      if (System.currentTimeMillis() - startReject > 2000) {
        startReject = System.currentTimeMillis();
      }
      if (shooter.getMainRPM() > 900) {
        indexer.setIndexerSpeed(0.5, 1);
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