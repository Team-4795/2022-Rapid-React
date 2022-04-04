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
      double rotation = Math.toDegrees(Math.atan2(robotPose.getY() - goalPose.getY(), robotPose.getX() - goalPose.getX()));
      double goalAngle = (robotPose.getRotation().getDegrees() + (180 - Math.abs(rotation)) * Math.signum(rotation)) % 360;
      double goalDistance = Units.metersToFeet(drivebase.getGoalPose().getTranslation().getDistance(drivebase.getPose().getTranslation()));

      double verticalSpeed = 0;
      double angularSpeed = 0;

      if (Math.abs(goalAngle) < 25 && (goalDistance < 6 || goalDistance > 8)) {
        isAligned = false;
        verticalSpeed = Math.signum(7 - goalDistance) * (Math.abs(goalDistance - 7) < 3 ? 0.5 : 1) * (Math.abs(goalAngle) > 10 ? 0.25 : 1);
      }

      if (Math.abs(goalAngle) > 4) {
        isAligned = false;
        angularSpeed = MathUtil.clamp(Math.abs(goalAngle / 50), 0.20, 0.7) * Math.signum(goalAngle);
      }

      drivebase.arcadeDrive(verticalSpeed, angularSpeed);
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