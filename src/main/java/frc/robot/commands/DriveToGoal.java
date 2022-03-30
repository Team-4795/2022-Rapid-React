// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivebase;

public class DriveToGoal extends CommandBase {
  private final Drivebase drivebase;
  private boolean isAligned;

  public DriveToGoal(Drivebase drivebase) {
    this.drivebase = drivebase;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    drivebase.enableBrakeMode();
    isAligned = false;
  }

  @Override
  public void execute() {
    isAligned = true;

    if (drivebase.hasGoalPose()) {
      var robotPose = drivebase.getPose();
      var goalPose = drivebase.getGoalPose();
      double goalAngle = (robotPose.getRotation().getDegrees() + (180 - Math.toDegrees(Math.atan2(robotPose.getY() - goalPose.getY(), robotPose.getX() - goalPose.getX())))) % 360;

      if (Math.abs(goalAngle) > 5) {
        isAligned = false;
        drivebase.arcadeDrive(0, MathUtil.clamp(Math.abs(goalAngle / 50), 0.125, 1) * Math.signum(goalAngle));
      } else if (Units.metersToFeet(drivebase.getGoalPose().getTranslation().getDistance(drivebase.getPose().getTranslation())) > 6) {
        isAligned = false;
        drivebase.arcadeDrive(-1, 0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.disableBrakeMode();
  }

  @Override
  public boolean isFinished() {
    return isAligned;
  }
}