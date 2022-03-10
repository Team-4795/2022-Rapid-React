// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Indexer;
import frc.robot.sensors.ColorSensor.Color;

public class BallManager extends CommandBase {
  private final Intake intake;
  private final Indexer indexer;
  private Alliance alliance;
  private long lastIncorrect;

  public BallManager(Superstructure superstructure) {
    this.intake = superstructure.intake;
    this.indexer = superstructure.indexer;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    alliance = DriverStation.getAlliance();
    lastIncorrect = 0;
  }

  @Override
  public void execute() {
    Color lowerColor = indexer.getLowerColor();

    if (System.currentTimeMillis() - lastIncorrect < 1000) {
      intake.setSpeed(0);
      indexer.setIndexerSpeed(0, -1);

      if (intake.isExtended()) intake.toggle();
    } else if ((lowerColor == Color.Red && alliance == Alliance.Blue) || (lowerColor == Color.Blue && alliance == Alliance.Red)) {
      lastIncorrect = System.currentTimeMillis();
    } else if (intake.isExtended()) {
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
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}