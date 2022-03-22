// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.sensors.ColorSensor.Color;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class BallManager extends CommandBase {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private Alliance alliance;

  public BallManager(Superstructure superstructure) {
    this.intake = superstructure.intake;
    this.indexer = superstructure.indexer;
    this.shooter = superstructure.shooter;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    alliance = DriverStation.getAlliance();
  }

  @Override
  public void execute() {
    if (intake.isExtended()) {
      double intakeSpeed = 0.75;
      double upperSpeed = 0.25;
      double lowerSpeed = 1;

      if (indexer.hasUpperBall()) {
        upperSpeed = 0;

        if (indexer.hasLowerBall()) intake.toggle();
      }

      intake.setSpeed(intakeSpeed);
      indexer.setIndexerSpeed(upperSpeed, lowerSpeed);
    } else {
      intake.setSpeed(0);
      indexer.setIndexerSpeed(0, 0);
    }

    Color upperColor = indexer.getUpperColor();

    if ((upperColor == Color.Red && alliance == Alliance.Blue) || (upperColor == Color.Blue && alliance == Alliance.Red)) {
      shooter.setShooterRPM(1000, 1000);

      if (shooter.getMainRPM() > 900) {
        indexer.setIndexerSpeed(0.5, 1);
      }
    } else {
      shooter.setShooterPower(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}