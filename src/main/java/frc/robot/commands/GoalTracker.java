// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Vision;

public class GoalTracker extends CommandBase {
  private final Drivebase drivebase;
  private final Vision vision;

  public GoalTracker(Drivebase drivebase, Vision vision) {
    this.drivebase = drivebase;
    this.vision = vision;

    addRequirements(vision);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d robotPose = drivebase.getPose();
    Pose2d goalPose = drivebase.getGoalPose();
    double rotation = Math.toDegrees(Math.atan2(robotPose.getY() - goalPose.getY(), robotPose.getX() - goalPose.getX()));
    double goalAngle = (robotPose.getRotation().getDegrees() + (180 - Math.abs(rotation)) * Math.signum(rotation)) % 360;
    double goalDistance = Units.metersToFeet(drivebase.getGoalPose().getTranslation().getDistance(drivebase.getPose().getTranslation()));

    if (Math.abs(goalAngle) < Math.min(20 + goalDistance * 1.5, 45) && goalDistance > 5 && goalDistance < 14) {
      vision.enableLED();

      if (vision.hasTarget() && drivebase.getAngularVelocity() < 30) {
        double distance = Units.feetToMeters(vision.getTargetDistance() + 1.5 + 2);
        double angle = vision.getTargetAngle();
        double transformRotation = drivebase.getPose().getRotation().getDegrees() - angle;
        double newX = 16.4592 / 2.0 - distance * Math.sin(Math.toRadians(transformRotation));
        double newY = 8.2296 / 2.0 + distance * Math.cos(Math.toRadians(transformRotation));
        
        drivebase.resetOdometry(new Pose2d(newX, newY, drivebase.getPose().getRotation()));
      }
    } else {
      vision.disableLED();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}