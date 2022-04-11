// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    if (drivebase.hasGoalPose()) {
      Pose2d robotPose = drivebase.getPose();
      Pose2d goalPose = drivebase.getGoalPose();
      double rotation = Math.toDegrees(Math.atan2(robotPose.getY() - goalPose.getY(), robotPose.getX() - goalPose.getX()));
      double goalAngle = (robotPose.getRotation().getDegrees() + (180 - Math.abs(rotation)) * Math.signum(rotation)) % 360;
      double goalDistance = Units.metersToFeet(drivebase.getGoalPose().getTranslation().getDistance(drivebase.getPose().getTranslation()));

      if (Math.abs(goalAngle) < 25 && goalDistance > 6 && goalDistance < 13) {
        vision.enableLED();

        double speed = (drivebase.getWheelSpeeds().leftMetersPerSecond + drivebase.getWheelSpeeds().rightMetersPerSecond) / 2.0;

        if (vision.hasTarget() && drivebase.getAngularVelocity() < 30 && (speed < 1.5 || Math.abs(goalAngle) < 10)) {
          double distance = vision.getTargetDistance();
          double angle = vision.getTargetAngle();
          
          drivebase.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(angle)));
          drivebase.setGoalPose(new Pose2d(Units.feetToMeters(distance + 1.5), 0, Rotation2d.fromDegrees(0)));
        }
      } else {
        vision.disableLED();
      }
    } else {
      vision.enableLED();

      if (vision.hasTarget()) {
        double distance = vision.getTargetDistance();
        double angle = vision.getTargetAngle();
        
        drivebase.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(angle)));
        drivebase.setGoalPose(new Pose2d(Units.feetToMeters(distance + 1.5), 0, Rotation2d.fromDegrees(0)));
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