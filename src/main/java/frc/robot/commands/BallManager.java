// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.sensors.Colors;

public class BallManager extends CommandBase {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Alliance alliance = DriverStation.getAlliance();
  private boolean ejectBall = false;
  private boolean ejectUpper = false;
  private long ejectionStart = 0;

  public BallManager(Intake intake, Indexer indexer, Shooter shooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    
    addRequirements(intake, indexer);
  }

  private void eject(boolean ejectUpper) {
    ejectBall = true;
    this.ejectUpper = ejectUpper;
    ejectionStart = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    if (intake.isExtended()) {
      if (ejectBall) {
        intake.setSpeed(-0.5);
        indexer.setIndexerSpeed(ejectUpper ? -0.5 : 0, -0.5);

        if (System.currentTimeMillis() - ejectionStart > 1000 * 3) {
          ejectBall = false;
          ejectUpper = false;
        }

        return;
      }

      Colors upperColor = indexer.getUpperColor();
      Colors lowerColor = indexer.getLowerColor();
      double intakeSpeed = 0.5;
      double upperSpeed = 0;
      double lowerSpeed = 0;

      switch (upperColor) {
        case Other:
          upperSpeed = 0.5;
          break;
        case Red:
          if (alliance == Alliance.Red) {
            upperSpeed = 0;
          } else {
            eject(true);
          }
          break;
        case Blue:
          if (alliance == Alliance.Blue) {
            upperSpeed = 0.5;
          } else {
            eject(true);
          }
          break;
      }

      switch (lowerColor) {
        case Other:
          lowerSpeed = 0.5;
          break;
        case Red:
          if (alliance == Alliance.Red) {
            lowerSpeed = 0.5;
          } else {
            eject(false);
          }
          break;
        case Blue:
          if (alliance == Alliance.Blue) {
            lowerSpeed = 0.5;
          } else {
            eject(false);
          }
          break;
      }

      if (upperSpeed == 0 && lowerColor != Colors.Other) lowerSpeed = 0;

      intake.setSpeed(intakeSpeed);
      indexer.setIndexerSpeed(upperSpeed, lowerSpeed);
    } else {
      intake.setSpeed(0);

      if (shooter.getShooterMainRPM() > 4000) {
        indexer.setIndexerSpeed(0.5, 0.5);
      } else {
        indexer.setIndexerSpeed(0, 0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
